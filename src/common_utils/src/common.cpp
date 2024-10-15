#include <common_utils/common.h>
#include <iostream>
using namespace std;


Interpolator::Interpolator() {}
Interpolator::Interpolator(const MatrixXf &points, const VectorXf &values) : points(points), values(values) {}

float Interpolator::interpolate(const VectorXf point) {
    float eps = 0.00001;
    int n = points.rows();
    VectorXf weight(n);
    for (int i = 0; i < n; ++i) {
        auto distance = (points.row(i) - point.transpose()).norm();
        weight(i) = 1 / (eps + pow(static_cast<float>(distance), 4));
    }
    weight = weight.normalized();
    float res = weight.dot(values);
    return res;
}

Interpolator &Interpolator::operator=(const Interpolator &other) {
    if (this == &other) {
        return *this;
    }
    points = other.points;
    values = other.values;
    return *this;
}