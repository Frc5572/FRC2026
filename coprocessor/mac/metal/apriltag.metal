#include <metal_stdlib>
using namespace metal;

kernel void colorToGray(
    device const uchar *rgb [[buffer(0)]],
    device uchar *gray [[buffer(1)]],
    constant int &rows [[buffer(2)]],
    constant int &cols [[buffer(3)]],
    uint2 gid [[thread_position_in_grid]]) {
        int col = gid.x;
        int row = gid.y;

        if (col >= cols || row > rows) return;

        int gray_offset = row * cols + col;
        int rgb_offset = gray_offset * 3;

        uchar r = rgb[rgb_offset + 0];
        uchar g = rgb[rgb_offset + 1];
        uchar b = rgb[rgb_offset + 2];

        gray[gray_offset] = uchar(0.299f * r + 0.587f * g + 0.114f * b);
    }

kernel void decimate(
    device const uchar *gray_image [[buffer(0)]],
    device uchar *decimate_image [[buffer(1)]],
    constant int &width [[buffer(2)]],
    constant int &height [[buffer(3)]],
    uint gid [[thread_position_in_grid]]) {
        int idx = gid;
        int total = width * height;

        if (idx >= total / 4) return;

        int base_idx = idx * 4;
        int row = base_idx / width;
        int col = base_idx % width;
        if (col + 1 < width && row + 1 < height) {
            int tl = row * width + col;
            int tr = row * width + (col + 1);
            int bl = (row + 1) * width + col;
            int br = (row + 1) * width + (col + 1);

            int avg = (int(gray_image[tl]) +
                    int(gray_image[tr]) +
                    int(gray_image[bl]) +
                    int(gray_image[br])) / 4;

            decimate_image[idx] = uchar(avg)
        }
    }