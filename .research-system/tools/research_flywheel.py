#!/usr/bin/env python3
"""HiAgent 2: local research state, task leases, and evidence-aware handoffs."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime, timedelta, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import sys
import tempfile
import uuid

SYSTEM = Path(__file__).resolve().parents[1]
VERSION = '2.0'
DEPENDENCIES = {'depends_on', 'tested_by', 'implemented_by', 'executed_as',
                'produces', 'visualized_by', 'stated_in'}


class ResearchError(Exception):
    pass


def now():
    return datetime.now(timezone.utc).isoformat(timespec='seconds').replace('+00:00', 'Z')


def timestamp(value):
    try:
        if not isinstance(value, str) or not value.endswith('Z'):
            raise ValueError()
        return datetime.fromisoformat(value.replace('Z', '+00:00'))
    except (ValueError, TypeError):
        raise ResearchError(f'invalid UTC timestamp: {value}') from None


def read_json(path):
    def reject(value):
        raise ResearchError(f'non-finite JSON number: {value}')
    return json.loads(Path(path).read_text(encoding='utf-8'), parse_constant=reject)


def atomic_write(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    mode = path.stat().st_mode & 0o777 if path.exists() else 0o644
    fd, temporary = tempfile.mkstemp(dir=path.parent, prefix='.hiagent-')
    try:
        with os.fdopen(fd, 'wb') as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary, mode)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def write_json(path, value):
    atomic_write(path, (json.dumps(value, ensure_ascii=False, indent=2,
                                  sort_keys=True, allow_nan=False) + '\n').encode())


def project_file(root, relative, exists=False):
    if not isinstance(relative, str) or not relative or '\\' in relative:
        raise ResearchError(f'invalid project-relative path: {relative}')
    parts = PurePosixPath(relative)
    if parts.is_absolute() or '..' in parts.parts or str(parts) == '.':
        raise ResearchError(f'path escapes project: {relative}')
    if str(parts) != relative:
        raise ResearchError(f'path must use its canonical spelling: {relative}')
    cursor = Path(root)
    for part in parts.parts:
        cursor = cursor / part
        if cursor.is_symlink():
            raise ResearchError(f'symlink component: {relative}')
    if exists and not cursor.is_file():
        raise ResearchError(f'artifact missing or not a regular file: {relative}')
    return cursor


def digest(path):
    with Path(path).open('rb') as handle:
        return hashlib.file_digest(handle, 'sha256').hexdigest()


@contextmanager
def project_lock(root):
    # ponytail: one local writer; a database is needed for multi-host coordination.
    fd = os.open(root, os.O_RDONLY)
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ResearchError('another state writer is active; retry after it finishes') from None
        yield
    finally:
        os.close(fd)


def schema_check(value, schema, location='document'):
    """Only the JSON Schema subset actually used by our bundled schemas."""
    types = {'object': lambda x: isinstance(x, dict), 'array': lambda x: isinstance(x, list),
             'string': lambda x: isinstance(x, str), 'null': lambda x: x is None,
             'integer': lambda x: type(x) is int, 'boolean': lambda x: type(x) is bool}
    expected = schema.get('type')
    if expected and not any(types[t](value) for t in ([expected] if isinstance(expected, str) else expected)):
        raise ResearchError(f'{location}: wrong type')
    if 'const' in schema and value != schema['const']:
        raise ResearchError(f'{location}: expected {schema["const"]}')
    if 'enum' in schema and value not in schema['enum']:
        raise ResearchError(f'{location}: unsupported value {value}')
    if isinstance(value, str):
        if len(value) < schema.get('minLength', 0):
            raise ResearchError(f'{location}: empty string')
        if 'pattern' in schema and not re.search(schema['pattern'], value):
            raise ResearchError(f'{location}: invalid format')
    if type(value) is int and value < schema.get('minimum', value):
        raise ResearchError(f'{location}: below minimum')
    if isinstance(value, dict):
        props = schema.get('properties', {})
        if set(schema.get('required', [])) - value.keys():
            raise ResearchError(f'{location}: required fields missing')
        if schema.get('additionalProperties') is False and value.keys() - props.keys():
            raise ResearchError(f'{location}: unexpected fields')
        for key in value.keys() & props.keys():
            schema_check(value[key], props[key], f'{location}.{key}')
    if isinstance(value, list):
        for index, item in enumerate(value):
            schema_check(item, schema.get('items', {}), f'{location}[{index}]')


def config():
    agents = {item['id']: item for item in read_json(SYSTEM / 'config/agents.json')['agents']}
    return agents, read_json(SYSTEM / 'config/models.json')


def profile(root):
    path = project_file(root, '.research/project.json')
    return read_json(path) if path.exists() else {'layout': {}, 'budget': {}}


def scopes(root, agent):
    context = profile(root)
    layout = context.get('layout', {})
    mapping = {'src/': 'source', 'tests/': 'tests', 'paper/': 'paper',
               'runs/': 'results', 'experiments/specs/': 'protocols', 'data/': 'data'}
    def extend(paths):
        expanded = list(paths)
        for path in paths:
            expanded.extend(layout.get(mapping.get(path), []))
        return list(dict.fromkeys(expanded))
    reads = extend(agent['read_paths']) + context.get('context_files', [])
    if agent['id'] == 'research-governor':
        reads += [path for group in layout.values() for path in group]
    return {'read': list(dict.fromkeys(reads)), 'write': extend(agent['write_paths'])}


def in_scope(path, allowed):
    return any(path == p.rstrip('/') or (p.endswith('/') and path.startswith(p)) for p in allowed)


def check_graph(root, graph):
    if isinstance(graph, dict) and graph.get('schema_version') == '1.0':
        raise ResearchError('legacy graph: run HiAgent upgrade with a backup first')
    schema_check(graph, read_json(SYSTEM / 'schemas/graph.schema.json'))
    timestamp(graph['created_at']); timestamp(graph['updated_at'])
    agents, models = config()
    nodes = {}
    for node in graph['nodes']:
        key = node['id']
        if key in nodes:
            raise ResearchError(f'duplicate node: {key}')
        if node['owner_agent'] not in agents or node['model_tier'] not in models['tiers']:
            raise ResearchError(f'unknown agent or model tier: {key}')
        agent = agents[node['owner_agent']]
        if node['type'] not in agent['accepts_node_types']:
            raise ResearchError(f'agent cannot own this node type: {key}')
        if node['model_tier'] not in agent['allowed_model_tiers']:
            raise ResearchError(f'model tier not allowed for agent: {key}')
        timestamp(node['created_at']); timestamp(node['updated_at'])
        access = scopes(root, agent)
        for direction, field in [('read', 'input_artifacts'), ('write', 'output_artifacts')]:
            for path in node[field]:
                project_file(root, path)
                if not in_scope(path, access[direction]):
                    raise ResearchError(f'{path} outside {direction} scope of {key}')
        if len(node['acceptance']) != len(set(node['acceptance'])):
            raise ResearchError(f'duplicate acceptance criteria: {key}')
        attempts = node['metadata'].get('attempts', [])
        if not isinstance(attempts, list) or not all(isinstance(a, dict) for a in attempts):
            raise ResearchError(f'invalid attempts: {key}')
        lease = node['metadata'].get('lease')
        if node['status'] == 'running' and not lease:
            raise ResearchError(f'running node has no lease: {key}')
        if lease:
            if node['status'] != 'running' or not isinstance(lease, dict):
                raise ResearchError(f'lease/state mismatch: {key}')
            if not {'attempt_id', 'snapshot', 'expires_at', 'model'} <= lease.keys():
                raise ResearchError(f'incomplete lease: {key}')
            timestamp(lease['expires_at'])
        nodes[key] = node
    adjacency = {key: [] for key in nodes}
    seen = set()
    for edge in graph['edges']:
        identity = (edge['from'], edge['to'], edge['type'])
        if identity in seen or edge['from'] not in nodes or edge['to'] not in nodes:
            raise ResearchError('duplicate edge or missing endpoint')
        seen.add(identity)
        if edge['type'] in DEPENDENCIES:
            adjacency[edge['from']].append(edge['to'])
    indegree = {key: 0 for key in nodes}
    for children in adjacency.values():
        for child in children:
            indegree[child] += 1
    queue = [key for key, count in indegree.items() if count == 0]
    visited = 0
    while queue:
        parent = queue.pop(); visited += 1
        for child in adjacency[parent]:
            indegree[child] -= 1
            if indegree[child] == 0:
                queue.append(child)
    if visited != len(nodes):
        raise ResearchError('dependency cycle')
    return nodes


def load_graph(root):
    graph = read_json(project_file(root, '.research/graph.json', True))
    check_graph(root, graph)
    return graph


def commit(root, before, graph, event, expected=None):
    if expected is not None and before['revision'] != expected:
        raise ResearchError(f'revision conflict: expected {expected}, current {before["revision"]}')
    graph['revision'] = before['revision'] + 1
    graph['updated_at'] = now()
    check_graph(root, graph)
    event = dict(event, revision=graph['revision'], timestamp=graph['updated_at'])
    # graph.json is authoritative; checkpoints include the old graph and proposed event.
    checkpoint = project_file(root, f'.research/checkpoints/revision-{before["revision"]}.json')
    write_json(checkpoint, {'graph': before, 'transaction': event})
    events = project_file(root, '.research/events.jsonl')
    with events.open('a', encoding='utf-8') as handle:
        handle.write(json.dumps(event, ensure_ascii=False) + '\n')
        handle.flush(); os.fsync(handle.fileno())
    write_json(project_file(root, '.research/graph.json'), graph)
    return event


def output_manifest(node):
    artifacts = {}
    for attempt in node['metadata'].get('attempts', []):
        if attempt.get('node_version') == node['version']:
            for item in attempt.get('artifacts', []):
                artifacts[item['path']] = item['sha256']
    return artifacts


def descendants(graph, initial):
    queue = [(key, False) for key in initial]
    visited, affected = set(), set(initial)
    nodes = {n['id']: n for n in graph['nodes']}
    while queue:
        parent, evidence_return = queue.pop()
        if (parent, evidence_return) in visited:
            continue
        visited.add((parent, evidence_return))
        for edge in graph['edges']:
            if edge['from'] != parent or edge['type'] in {'targets', 'supersedes'}:
                continue
            if evidence_return and nodes[parent]['type'] == 'claim' and edge['type'] == 'tested_by':
                continue
            affected.add(edge['to'])
            queue.append((edge['to'], edge['type'] in {'supports', 'refutes'}))
    return affected


def integrity(root, graph):
    issues = []
    initial = {n['id'] for n in graph['nodes'] if n['freshness'] == 'stale'}
    for node in graph['nodes']:
        if node['status'] != 'completed':
            continue
        outputs = output_manifest(node)
        for path in node['output_artifacts']:
            if path not in outputs:
                issues.append({'node': node['id'], 'path': path, 'reason': 'unverified output'})
                initial.add(node['id'])
        snapshots = [a.get('snapshot', {}) for a in node['metadata'].get('attempts', [])
                     if a.get('node_version') == node['version']]
        hashes = dict(snapshots[-1].get('inputs', {})) if snapshots else {}
        hashes.update(outputs)
        for path, expected in hashes.items():
            try:
                reason = None if digest(project_file(root, path, True)) == expected else 'hash changed'
            except (OSError, ResearchError) as exc:
                reason = str(exc)
            if reason:
                issues.append({'node': node['id'], 'path': path, 'reason': reason})
                initial.add(node['id'])
    return issues, descendants(graph, initial)


def blockers(root, graph, node, stale):
    result = []
    nodes = {n['id']: n for n in graph['nodes']}
    if node['id'] in stale:
        result.append('stale evidence; revise or supersede this node')
    for edge in graph['edges']:
        if edge['to'] == node['id'] and edge['type'] in DEPENDENCIES:
            parent = nodes[edge['from']]
            if parent['status'] != 'completed' or parent['id'] in stale:
                result.append(f'dependency not current and completed: {parent["id"]}')
            if edge.get('requirement', 'completed') == 'supported' and parent['verdict'] != 'supported':
                result.append(f'dependency lacks supported verdict: {parent["id"]}')
    for path in node['input_artifacts']:
        try:
            project_file(root, path, True)
        except ResearchError as exc:
            result.append(str(exc))
    limit = profile(root).get('budget', {}).get('max_attempts_per_node', 3)
    if type(limit) is not int or limit < 1:
        raise ResearchError('max_attempts_per_node must be a positive integer')
    if len(node['metadata'].get('attempts', [])) >= limit:
        result.append('attempt budget exhausted; diagnose or create a justified new node')
    if node['type'] == 'run' and node['metadata'].get('attempts'):
        result.append('Run history is immutable; use a new node and output directory for another execution')
    return result


def snapshot(root, graph, node):
    nodes = {n['id']: n for n in graph['nodes']}
    paths, dependencies = set(node['input_artifacts']), {}
    for edge in graph['edges']:
        if edge['to'] == node['id'] and edge['type'] in DEPENDENCIES:
            parent = nodes[edge['from']]
            dependencies[parent['id']] = {key: parent[key] for key in ['version', 'status', 'verdict', 'freshness']}
            paths.update(parent['output_artifacts'])
    return {'node_version': node['version'], 'dependencies': dependencies,
            'inputs': {path: digest(project_file(root, path, True)) for path in sorted(paths)}}


def packet(root, graph, node):
    agents, models = config()
    agent = agents[node['owner_agent']]
    return {'available': True, 'revision': graph['revision'], 'node': deepcopy(node),
            'agent': {**agent, 'effective_scopes': scopes(root, agent)},
            'required_skills': list(dict.fromkeys(agent['required_skills'] + node['required_skills'])),
            'model': models['tiers'][node['model_tier']], 'model_verified': False,
            'project': profile(root), 'snapshot': snapshot(root, graph, node),
            'recent_attempts': node['metadata'].get('attempts', [])[-3:],
            'scientific_rule': 'Completion does not imply scientific support. Check verdict, scope and evidence.',
            'human_gate': node['type'] == 'decision'}


def route(root, graph=None, node_id=None):
    graph = graph or load_graph(root)
    _, stale = integrity(root, graph)
    waiting = {}
    candidates = sorted(graph['nodes'], key=lambda n: (-n['priority'], n['created_at'], n['id']))
    if node_id:
        candidates = [n for n in candidates if n['id'] == node_id]
        if not candidates:
            raise ResearchError(f'unknown node: {node_id}')
    for node in candidates:
        if node['status'] not in {'ready', 'partial'}:
            continue
        reasons = blockers(root, graph, node, stale)
        if reasons:
            waiting[node['id']] = reasons
        else:
            return packet(root, graph, node)
    return {'available': False, 'revision': graph['revision'], 'blockers': waiting,
            'reason': 'No executable node; inspect running leases, proposed plans and blockers.'}


def status(root):
    graph = load_graph(root)
    issues, stale = integrity(root, graph)
    active = []
    for node in graph['nodes']:
        lease = node['metadata'].get('lease')
        if lease:
            active.append({'node_id': node['id'], **lease,
                           'expired': timestamp(lease['expires_at']) <= timestamp(now())})
    return {'project_id': graph['project_id'], 'revision': graph['revision'],
            'nodes': [{key: n[key] for key in ['id', 'title', 'status', 'verdict', 'freshness']} for n in graph['nodes']],
            'unsupported_claims': [n['id'] for n in graph['nodes'] if n['type'] == 'claim'
                                   and (n['verdict'] != 'supported' or n['id'] in stale)],
            'stale_nodes': sorted(stale), 'integrity_issues': issues, 'active_attempts': active,
            'budget': profile(root).get('budget', {}),
            'usage': {'completed_attempts': sum(len(n['metadata'].get('attempts', [])) for n in graph['nodes']),
                      'model_tokens': None, 'gpu_hours': None}}


def claim(root, node_id=None, expected=None, tier=None):
    with project_lock(root):
        before = load_graph(root); graph = deepcopy(before)
        routed = route(root, graph, node_id)
        if not routed['available']:
            raise ResearchError(json.dumps(routed, ensure_ascii=False))
        node = next(n for n in graph['nodes'] if n['id'] == routed['node']['id'])
        if node['type'] == 'decision':
            raise ResearchError('human decision required; record the actual user decision with decide')
        if not node['acceptance'] or not node['output_artifacts']:
            raise ResearchError('define checks and output artifacts before dispatch')
        if tier:
            node['model_tier'] = tier
            check_graph(root, graph)
        for other in graph['nodes']:
            if other['status'] == 'running' and set(node['output_artifacts']).intersection(other['output_artifacts']):
                raise ResearchError(f'write conflict with running node: {other["id"]}')
            if node['type'] == 'run' and other['id'] != node['id'] and other['type'] == 'run':
                if set(node['output_artifacts']).intersection(other['output_artifacts']):
                    raise ResearchError('Run output paths must be unique across nodes')
        if node['type'] == 'run' and any(project_file(root, p).exists() for p in node['output_artifacts']):
            raise ResearchError('Run outputs already exist; inspect them and create a fresh Run')
        seconds = profile(root).get('budget', {}).get('lease_seconds', 3600)
        if type(seconds) is not int or not 1 <= seconds <= 86400:
            raise ResearchError('lease_seconds must be between 1 and 86400')
        lease = {'attempt_id': uuid.uuid4().hex, 'snapshot': snapshot(root, graph, node),
                 'model': config()[1]['tiers'][node['model_tier']], 'runtime_model': None,
                 'runtime_evidence': None, 'created_at': now(),
                 'expires_at': (datetime.now(timezone.utc) + timedelta(seconds=seconds)).isoformat(timespec='seconds').replace('+00:00', 'Z')}
        node['metadata']['lease'] = lease
        node['status'] = 'running'; node['updated_at'] = now()
        commit(root, before, graph, {'event': 'claim', 'node_id': node['id'], 'attempt_id': lease['attempt_id']}, expected)
        return packet(root, graph, node)


def active_node(graph, node_id, attempt_id):
    node = next((n for n in graph['nodes'] if n['id'] == node_id), None)
    if not node or node['status'] != 'running' or node['metadata'].get('lease', {}).get('attempt_id') != attempt_id:
        raise ResearchError('no matching active attempt; claim the node before submitting')
    return node


def check_snapshot(root, graph, node):
    _, stale = integrity(root, graph)
    reasons = blockers(root, graph, node, stale)
    actual = snapshot(root, graph, node)
    expected = deepcopy(node['metadata']['lease']['snapshot'])
    for path in node['output_artifacts']:
        actual['inputs'].pop(path, None)
        expected['inputs'].pop(path, None)
    if reasons or actual != expected:
        raise ResearchError('dependencies or input snapshot changed: ' + '; '.join(reasons))


def activate(root, graph):
    activated = []
    _, stale = integrity(root, graph)
    for child in graph['nodes']:
        if child['status'] == 'proposed' and child['metadata'].get('approved_for_execution') is True:
            if not blockers(root, graph, child, stale):
                child['status'] = 'ready'
                child['updated_at'] = now()
                activated.append(child['id'])
    return activated


def apply_outcome(root, path, expected=None):
    relative = Path(os.path.abspath(path)).relative_to(Path(root).absolute()).as_posix()
    if not relative.startswith('.research/inbox/'):
        raise ResearchError('outcome must be inside .research/inbox')
    outcome = read_json(project_file(root, relative, True))
    schema_check(outcome, read_json(SYSTEM / 'schemas/outcome.schema.json'))
    timestamp(outcome['created_at'])
    with project_lock(root):
        before = load_graph(root); graph = deepcopy(before)
        node = active_node(graph, outcome['node_id'], outcome['attempt_id'])
        lease = node['metadata']['lease']
        if outcome['agent_id'] != node['owner_agent'] or node['type'] == 'decision':
            raise ResearchError('agent mismatch or human decision node')
        if node['type'] == 'run' and outcome['status'] == 'partial':
            raise ResearchError('active Runs retain their lease: record progress and renew; use a final outcome after execution stops')
        if timestamp(lease['expires_at']) <= timestamp(now()):
            raise ResearchError('attempt expired; inspect the process, then renew the same attempt for a late result or release abandoned work')
        check_snapshot(root, graph, node)
        if outcome['verdict'] != 'not_assessed':
            if node['owner_agent'] != 'evidence-auditor' or node['type'] not in {'claim', 'finding'}:
                raise ResearchError('only evidence-auditor can assign scientific verdicts to claims or findings')
            if outcome['status'] != 'completed':
                raise ResearchError('a scientific verdict requires a completed audit')
        artifacts = outcome['artifacts']; paths = [a['path'] for a in artifacts]
        if len(paths) != len(set(paths)):
            raise ResearchError('duplicate outcome artifact')
        access = scopes(root, config()[0][node['owner_agent']])
        for item in artifacts:
            if not in_scope(item['path'], access['write']) or item['path'] not in node['output_artifacts']:
                raise ResearchError('artifact outside declared task outputs or owner write scope')
            if digest(project_file(root, item['path'], True)) != item['sha256']:
                raise ResearchError('artifact hash mismatch')
        if outcome['status'] == 'completed':
            if not node['acceptance'] or not node['output_artifacts']:
                raise ResearchError('completion requires declared checks and output artifacts')
            if set(paths) != set(node['output_artifacts']):
                raise ResearchError('missing declared output artifacts')
            checks = outcome['checks']
            if len({c['name'] for c in checks}) != len(checks):
                raise ResearchError('duplicate outcome check')
            for required in node['acceptance']:
                check = next((c for c in checks if c['name'] == required), None)
                if not check or check['status'] != 'passed':
                    raise ResearchError(f'missing passed acceptance check: {required}')
                if not any(check['evidence'] == p or check['evidence'].startswith(p + ':')
                           or check['evidence'].startswith(p + '#') for p in paths):
                    raise ResearchError('check evidence must reference a hashed output artifact')
            if any(c['status'] == 'failed' for c in checks):
                raise ResearchError('failed check in completed outcome')
        runtime = lease.get('runtime_model')
        receipt = lease.get('runtime_evidence')
        if receipt and digest(project_file(root, receipt['path'], True)) != receipt['sha256']:
            raise ResearchError('runtime receipt changed after recording')
        node['metadata'].setdefault('attempts', []).append({**outcome, 'node_version': node['version'],
             'snapshot': lease['snapshot'], 'requested_model': lease['model'],
             'runtime_model': runtime, 'runtime_evidence': lease.get('runtime_evidence'),
             'runtime_verified': runtime is not None})
        del node['metadata']['lease']
        node['metadata']['next_actions'] = outcome['next_actions']
        node['metadata']['blockers'] = outcome['blockers']
        node['status'] = outcome['status']; node['verdict'] = outcome['verdict']
        node['freshness'] = 'current'; node['updated_at'] = now()
        if node['type'] == 'finding' and node['verdict'] in {'supported', 'refuted', 'insufficient_evidence', 'invalid_run'}:
            for edge in graph['edges']:
                if edge['from'] != node['id'] or edge['type'] not in {'supports', 'refutes'}:
                    continue
                target = next(n for n in graph['nodes'] if n['id'] == edge['to'])
                if target['type'] != 'claim' or target['status'] != 'completed':
                    continue
                if not target['output_artifacts'] or not set(target['output_artifacts']) <= lease['snapshot']['inputs'].keys():
                    raise ResearchError('audit must snapshot the exact Claim artifacts before changing its verdict')
                if edge['type'] == 'supports' and node['verdict'] == 'refuted':
                    raise ResearchError('refuted finding cannot use a supports edge')
                if edge['type'] == 'refutes' and node['verdict'] == 'supported':
                    raise ResearchError('supported finding cannot use a refutes edge')
                target['verdict'] = node['verdict']
                target['metadata']['audited_by'] = {'finding': node['id'], 'attempt_id': outcome['attempt_id']}
        activated = activate(root, graph)
        result = commit(root, before, graph, {'event': 'apply_outcome', 'node_id': node['id'],
                        'attempt_id': outcome['attempt_id'], 'activated': activated}, expected)
        result['runtime_verified'] = runtime is not None
        return result


def state_action(root, action, node_id=None, expected=None, reason='', attempt_id=None,
                 plan_path=None, decision=None, runtime_path=None):
    with project_lock(root):
        before = load_graph(root); graph = deepcopy(before)
        if expected is not None and expected != before['revision']:
            raise ResearchError('revision conflict')
        node = next((n for n in graph['nodes'] if n['id'] == node_id), None)
        if action not in {'plan', 'refresh'} and not node:
            raise ResearchError(f'unknown node: {node_id}')
        if action == 'plan':
            plan = read_json(project_file(root, plan_path, True))
            if not isinstance(plan, dict) or set(plan) != {'nodes', 'edges'}:
                raise ResearchError('plan requires nodes and edges arrays')
            for new in plan['nodes']:
                if new.get('status') not in {'proposed', 'ready'} or new.get('verdict') != 'not_assessed':
                    raise ResearchError('new tasks cannot fabricate completion or scientific verdicts')
                if new.get('metadata', {}).get('attempts') or new.get('metadata', {}).get('lease'):
                    raise ResearchError('new tasks cannot inject attempts or leases')
            graph['nodes'].extend(plan['nodes']); graph['edges'].extend(plan['edges'])
        elif action in {'refresh', 'invalidate'}:
            affected = integrity(root, graph)[1] if action == 'refresh' else descendants(graph, {node_id})
            for item in graph['nodes']:
                if item['id'] in affected:
                    item['freshness'] = 'stale'
                    item['metadata']['stale_reason'] = reason or 'artifact integrity changed'
        elif action == 'release':
            node = active_node(graph, node_id, attempt_id)
            lease = node['metadata'].pop('lease')
            node['metadata'].setdefault('attempts', []).append({'attempt_id': attempt_id,
                'node_version': node['version'], 'status': 'interrupted', 'reason': reason,
                'requested_model': lease['model'], 'snapshot': lease['snapshot'], 'created_at': now()})
            node['status'] = 'partial'
        elif action == 'renew':
            node = active_node(graph, node_id, attempt_id)
            if not reason.strip():
                raise ResearchError('renewal requires a process/recovery observation')
            check_snapshot(root, graph, node)
            seconds = profile(root).get('budget', {}).get('lease_seconds', 3600)
            if type(seconds) is not int or not 1 <= seconds <= 86400:
                raise ResearchError('lease_seconds must be between 1 and 86400')
            node['metadata']['lease']['expires_at'] = (datetime.now(timezone.utc) + timedelta(seconds=seconds)).isoformat(timespec='seconds').replace('+00:00', 'Z')
            node['metadata']['lease']['renewal_reason'] = reason
        elif action == 'progress':
            node = active_node(graph, node_id, attempt_id)
            if not plan_path or not plan_path.startswith('.research/inbox/'):
                raise ResearchError('progress file must be inside .research/inbox')
            progress = read_json(project_file(root, plan_path, True))
            if not isinstance(progress, dict) or progress.get('attempt_id') != attempt_id:
                raise ResearchError('progress belongs to another attempt')
            if not isinstance(progress.get('summary'), str) or not progress['summary'].strip():
                raise ResearchError('progress requires a summary and may include a process/job reference')
            node['metadata']['lease']['progress'] = {'path': plan_path,
                'sha256': digest(project_file(root, plan_path, True)), 'recorded_at': now(),
                'summary': progress['summary'], 'process': progress.get('process')}
        elif action == 'revise':
            if node['status'] == 'running' or (node['type'] == 'run' and node['metadata'].get('attempts')):
                raise ResearchError('release active work first; a Run retry requires a new node and unique outputs')
            for item in graph['nodes']:
                if item['id'] in descendants(graph, {node_id}) - {node_id}:
                    item['freshness'] = 'stale'
            node['version'] += 1
            node['status'] = 'ready'; node['verdict'] = 'not_assessed'; node['freshness'] = 'current'
            node['metadata']['revision_reason'] = reason
        elif action == 'decide':
            if node['type'] != 'decision' or node['status'] in {'completed', 'retired'}:
                raise ResearchError('not an open human decision')
            if decision not in {'approve', 'decline'} or not reason.strip():
                raise ResearchError('decision requires approve/decline and the actual user message')
            node['status'] = 'completed' if decision == 'approve' else 'blocked'
            record = {'decision': decision, 'user_message': reason, 'at': now()}
            node['metadata']['human_decision'] = record
            if len(node['output_artifacts']) != 1 or not node['output_artifacts'][0].startswith('.research/decisions/'):
                raise ResearchError('human decision needs one unique artifact under .research/decisions/')
            destination = project_file(root, node['output_artifacts'][0])
            if destination.exists():
                raise ResearchError('decision artifact already exists; preserve it and use a new decision node')
            write_json(destination, record)
            node['metadata'].setdefault('attempts', []).append({'attempt_id': uuid.uuid4().hex,
                'node_version': node['version'], 'status': node['status'], 'created_at': now(),
                'artifacts': [{'path': node['output_artifacts'][0], 'kind': 'json', 'sha256': digest(destination)}]})
        elif action == 'record-runtime':
            node = active_node(graph, node_id, attempt_id)
            receipt = read_json(project_file(root, runtime_path, True))
            required = {'attempt_id', 'model', 'reasoning_effort', 'session_id', 'source'}
            if not isinstance(receipt, dict) or not required <= receipt.keys() or receipt['attempt_id'] != attempt_id:
                raise ResearchError('runtime receipt is incomplete or belongs to another attempt')
            if receipt['source'] not in {'codex_runtime', 'provider_response'}:
                raise ResearchError('runtime evidence must come from the host/provider, not a worker guess')
            requested = node['metadata']['lease']['model']
            if receipt['model'] != requested['model'] or receipt['reasoning_effort'] != requested['reasoning_effort']:
                raise ResearchError('runtime model differs; release and reroute honestly')
            node['metadata']['lease']['runtime_model'] = requested
            node['metadata']['lease']['runtime_evidence'] = {'path': runtime_path,
                'sha256': digest(project_file(root, runtime_path, True)), 'session_id': receipt['session_id']}
        else:
            raise ResearchError('unknown state action')
        if node:
            node['updated_at'] = now()
        check_graph(root, graph)
        activated = activate(root, graph)
        return commit(root, before, graph, {'event': action, 'node_id': node_id, 'reason': reason,
                                          'activated': activated}, expected)


def main():
    parser = argparse.ArgumentParser(description='HiAgent research controller v2')
    sub = parser.add_subparsers(dest='command', required=True)
    for name in ['validate', 'status', 'resume', 'route', 'claim', 'refresh', 'apply-outcome',
                 'plan', 'release', 'renew', 'progress', 'revise', 'invalidate', 'decide', 'record-runtime']:
        item = sub.add_parser(name)
        item.add_argument('project_root')
        item.add_argument('--expected-revision', type=int)
        if name in {'route', 'claim', 'release', 'renew', 'progress', 'revise', 'invalidate', 'decide', 'record-runtime'}:
            item.add_argument('--node')
        if name == 'claim':
            item.add_argument('--tier')
        if name in {'apply-outcome', 'plan', 'record-runtime', 'progress'}:
            item.add_argument('file')
        if name in {'release', 'renew', 'record-runtime', 'progress'}:
            item.add_argument('--attempt-id', required=True)
        if name in {'release', 'renew', 'revise', 'invalidate', 'decide'}:
            item.add_argument('--reason', required=True)
        if name == 'decide':
            item.add_argument('--decision', choices=['approve', 'decline'], required=True)
    args = parser.parse_args(); root = Path(args.project_root).absolute()
    try:
        if args.command == 'validate':
            graph = load_graph(root); issues, stale = integrity(root, graph)
            result = {'valid': not issues and not stale, 'revision': graph['revision'],
                      'integrity_issues': issues, 'stale_nodes': sorted(stale)}
        elif args.command == 'status':
            result = status(root)
        elif args.command == 'resume':
            result = {'status': status(root), 'next': route(root)}
        elif args.command == 'route':
            result = route(root, node_id=args.node)
        elif args.command == 'claim':
            result = claim(root, args.node, args.expected_revision, args.tier)
        elif args.command == 'apply-outcome':
            result = apply_outcome(root, args.file, args.expected_revision)
        else:
            result = state_action(root, args.command, getattr(args, 'node', None), args.expected_revision,
                getattr(args, 'reason', ''), getattr(args, 'attempt_id', None),
                plan_path=getattr(args, 'file', None), decision=getattr(args, 'decision', None),
                runtime_path=getattr(args, 'file', None))
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 0 if result.get('valid', True) else 2
    except (ResearchError, OSError, ValueError, KeyError, TypeError) as exc:
        print(str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
