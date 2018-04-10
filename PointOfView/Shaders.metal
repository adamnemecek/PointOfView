#include <metal_stdlib>
using namespace metal;

constexpr constant static float pointRadius = 2.0e-3;

struct PointVertex {
    float4 clipPosition           [[position]];
    float3 viewPosition [[sample_perspective]];
    float3 pointViewPosition          [[flat]];
    float intensity                   [[flat]];
};

vertex PointVertex v_plotting(constant float    *xPositions  [[buffer(0)]],
                              constant float    *yPositions  [[buffer(1)]],
                              constant float    *zPositions  [[buffer(2)]],
                              constant uchar    *intensities [[buffer(3)]],
                              constant float4x4 &viewMatrix  [[buffer(4)]],
                              constant float4x4 &clipMatrix  [[buffer(5)]],
                              uint               vertexID    [[vertex_id]]) {
    PointVertex out;
    out.intensity = float(intensities[vertexID / 3]) / 255;
    out.pointViewPosition = (viewMatrix * float4(xPositions[vertexID / 3], yPositions[vertexID / 3], zPositions[vertexID / 3], 1)).xyz;
    
    float3 horizontal = normalize(cross(out.pointViewPosition, float3(0, 1, 0)));
    float3 vertical = normalize(cross(horizontal, out.pointViewPosition));
    switch (vertexID % 3) {
        case 0:
            out.viewPosition = out.pointViewPosition + vertical * pointRadius * 2;
            break;
        case 1:
            out.viewPosition = out.pointViewPosition - vertical * pointRadius + horizontal * pointRadius / sqrt(3.0) * 3;
            break;
        default:
            out.viewPosition = out.pointViewPosition + vertical * pointRadius + horizontal * pointRadius / sqrt(3.0) * 3;
    }
    
    out.clipPosition = clipMatrix * float4(out.viewPosition, 1);
    return out;
}

fragment float4 f_plotting(PointVertex in [[stage_in]]) {
    return float4(float3(in.intensity), 1);
}
