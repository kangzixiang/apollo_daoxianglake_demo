#pragma once

namespace demo {

// Jet colormap inspired by Matlab. Grayvalues are expected in the range [0, 1]
// and are converted to RGB values in the same range.
class JetColormap {
public:
    static float red(const float gray);
    static float green(const float gray);
    static float blue(const float gray);

private:
    static float interpolate(const float val, const float y0, const float x0,
                             const float y1, const float x1);
    static float base(const float val);
};

}  // namespace demo