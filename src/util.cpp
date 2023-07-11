#include "util.h"

namespace demo {

float JetColormap::red(const float gray) {
    return base(gray - 0.25f);
}

float JetColormap::green(const float gray) {
    return base(gray);
}

float JetColormap::blue(const float gray) {
    return base(gray + 0.25f);
}

float JetColormap::base(const float val) {
    if (val <= 0.125f) {
        return 0.0f;
    } else if (val <= 0.375f) {
        return interpolate(2.0f * val - 1.0f, 0.0f, -0.75f, 1.0f, -0.25f);
    } else if (val <= 0.625f) {
        return 1.0f;
    } else if (val <= 0.87f) {
        return interpolate(2.0f * val - 1.0f, 1.0f, 0.25f, 0.0f, 0.75f);
    } else {
        return 0.0f;
    }
}

float JetColormap::interpolate(const float val, const float y0, const float x0,
                               const float y1, const float x1) {
    return (val - x0) * (y1 - y0) / (x1 - x0) + y0;
}

}  // namespace demo
