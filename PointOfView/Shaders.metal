#include <metal_stdlib>
using namespace metal;

struct PointVertex {
    float4 clipPosition           [[position]];
    float3 viewPosition [[center_perspective]];
    float3 pointViewPosition          [[flat]];
    float  pointRadius                [[flat]];
    float3 pointColor                 [[flat]];
};

struct Uniforms {
    float4x4 viewMatrix;
    float4x4 clipMatrix;
    float3 pointColor;
    float pointRadius;
};

vertex PointVertex v_plotting(constant float    *xPositions  [[buffer(0)]],
                              constant float    *yPositions  [[buffer(1)]],
                              constant float    *zPositions  [[buffer(2)]],
                              constant uchar    *intensities [[buffer(3)]],
                              constant Uniforms &uniforms    [[buffer(4)]],
                              uint               vertexID    [[vertex_id]]) {
    PointVertex out;
    out.pointRadius = uniforms.pointRadius;
    out.pointColor = float(intensities[vertexID / 3]) / 255 * uniforms.pointColor;
    out.pointViewPosition = (uniforms.viewMatrix * float4(xPositions[vertexID / 3], yPositions[vertexID / 3], zPositions[vertexID / 3], 1)).xyz;
    
    float3 forward = normalize(out.pointViewPosition);
    float3 horizontal = normalize(cross(out.pointViewPosition, float3(0, 1, 0)));
    float3 vertical = normalize(cross(horizontal, out.pointViewPosition));
    switch (vertexID % 3) {
        case 0:
            out.viewPosition = out.pointViewPosition + vertical * out.pointRadius * 2;
            break;
        case 1:
            out.viewPosition = out.pointViewPosition - vertical * out.pointRadius + horizontal * out.pointRadius / sqrt(3.0) * 3;
            break;
        default:
            out.viewPosition = out.pointViewPosition - vertical * out.pointRadius - horizontal * out.pointRadius / sqrt(3.0) * 3;
    }
    
    out.pointViewPosition -= forward * out.pointRadius;
    out.clipPosition = uniforms.clipMatrix * float4(out.viewPosition, 1);
    return out;
}

fragment float4 f_plotting(PointVertex in [[stage_in]]) {
    float3 c = in.pointViewPosition;
    float3 d = normalize(in.viewPosition);
    float3 cd = c * d;
    float cds = cd.x + cd.y + cd.z;
    float cds2 = cds * cds;
    float3 d2 = d * d;
    float3 c2 = c * c;
    float d2s = d2.x + d2.y + d2.z;
    float c2s = c2.x + c2.y + c2.z;
    float radicand = max(cds2 - d2s * (c2s - in.pointRadius * in.pointRadius), 0.0);
    float factor = 2 * sqrt(radicand) / d2s;
    return float4(in.pointColor, pow(factor / in.pointRadius / 2, 2.0));
}
