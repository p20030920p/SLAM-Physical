#!/usr/bin/env bash
set -eo pipefail
# A per-session Mesa shader override, following the pinned Sim2Real compatibility
# approach. System ROS files are never modified.
RUN="$1"
shift
RVIZ_PREFIX="$(ros2 pkg prefix rviz_rendering)"
OVERRIDE="$RUN/rviz_override"
mkdir -p "$OVERRIDE/share/ament_index/resource_index/packages" "$OVERRIDE/share/rviz_rendering/ogre_media"
cp -a "$RVIZ_PREFIX/share/rviz_rendering/ogre_media/." "$OVERRIDE/share/rviz_rendering/ogre_media/"
touch "$OVERRIDE/share/ament_index/resource_index/packages/rviz_rendering"
SHADERS="$OVERRIDE/share/rviz_rendering/ogre_media/materials/glsl120"
cat > "$SHADERS/indexed_8bit_image.vert" <<'SHADER'
#version 420 compatibility
in vec4 uv0;
out vec2 UV;
void main() { gl_Position = ftransform(); UV = vec2(uv0); }
SHADER
cat > "$SHADERS/indexed_8bit_image.frag" <<'SHADER'
#version 420 compatibility
in vec2 UV;
layout(binding = 0) uniform sampler2D eight_bit_image;
layout(binding = 1) uniform sampler1D palette;
uniform float alpha;
void main() {
  vec4 color = texture(palette, 0.999 * texture(eight_bit_image, UV).x);
  gl_FragColor = vec4(color.rgb, color.a * alpha);
}
SHADER
export AMENT_PREFIX_PATH="$OVERRIDE:${AMENT_PREFIX_PATH:-}"
export QT_QPA_PLATFORM=xcb
exec rviz2 "$@"
