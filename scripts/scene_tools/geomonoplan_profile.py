"""Display defaults for opt-in GeoMonoPlan maps; no controller parameters."""
RENDER_ENV = {
    'ISAAC_VIEW_RENDERER': 'RaytracedLighting',
    'ISAAC_VIEW_ANTIALIASING': 'FXAA',
    'ISAAC_SUN_INTENSITY': '1500',
    'ISAAC_DOME_INTENSITY': '600',
    'ISAAC_AMBIENT_LIGHT_INTENSITY': '0.05',
}
# Unity's baked light probes are not included in exported meshes. This modest
# albedo-proportional material term keeps operational views readable while
# retaining texture, original direct lights, shadows and metric depth.
AMBIENT_TEXTURE_STRENGTH = .55
