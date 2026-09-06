---
name: literature-grounding
description: Conduct evidence-grounded literature review, novelty analysis, citation verification, related-work mapping, and baseline discovery. Use when scientific claims depend on external publications; do not use for general web summaries.
---

# Literature Grounding

## Source discipline

1. Prefer original papers, official proceedings, author repositories, and official dataset documentation.
2. Use search results and abstracts for discovery. Read the relevant full text before grounding a central novelty, method, or comparison claim.
3. Record title, authors, year, venue, DOI or canonical URL, access date, and an exact evidence location.
4. Label each statement direct, inferred, or not_found.
5. Keep conflicting evidence visible.
6. Never invent or repair a citation from memory.

## Novelty discipline

- Identify the closest work, not only broadly related work.
- State the exact overlap and remaining difference.
- Search for evidence that challenges the proposed novelty.
- Do not declare first, novel, or state of the art from search snippets or missing results.
- If evidence is insufficient, return insufficient literature evidence.

## Output

Write source and gap artifacts plus an outcome envelope. Do not edit manuscript prose or report experimental findings.

