#include <common_utils/common.h>
#include <iostream>
using namespace std;

Interpolator::Interpolator() {}

/**
 * @brief 构造一个Interpolartor对象，该对象用于通过控制点来插值其余点的结果，输入维度无限制
 * @details 
 * @param points_data 
 * @param values 
 * @param weights 
 */
Interpolator::Interpolator(const MatrixXf &points_data, const VectorXf &values, VectorXf &weights)
    : values(values), weights(weights) {
    // 初始化归一化后的points
    points_rows = points_data.rows();
    points_cols = points_data.cols();
    points.resize(points_rows, points_cols);
    normalization_params.resize(points_cols);
    for (int i = 0; i < points_cols; ++i) {
        // 获取当前行的最小值和最大值
        float min_val = points_data.col(i).minCoeff();
        float max_val = points_data.col(i).maxCoeff();
        float range = max_val - min_val;

        // 保存最小值和跨度到normalization_params中
        normalization_params[i] = {min_val, range};

        // 对每个值进行归一化处理
        for (int j = 0; j < points_rows; ++j) {
            points(j, i) = (points_data(j, i) - min_val) / range;
        }
    }
    cout << "Normalized reference points: " << endl << points << endl;
}

/**
 * @brief 该函数用于插值，根据类初始化时使用的参考点计算出某点的插值结果
 * @details 输入维度无限制，输出维度为1
 * @param point 
 * @return float 
 */
float Interpolator::interpolate(VectorXf point) {
    // 先正则化，然后再使用weights来加强某些维度的比重
    for (int i = 0; i < points_cols; i++) {
        point[i] = (point[i] - normalization_params[i].first) / normalization_params[i].second;
    }
    float eps = 0.00001;
    int n = points_rows;
    VectorXf point_weight(n);// 每个点的权重，和维度的权重weights不同
    for (int i = 0; i < n; ++i) {
        auto distance = weights.cwiseProduct((points.row(i).transpose() - point)).norm();
        point_weight(i) = 1 / (eps + pow(static_cast<float>(distance), 4));
    }
    point_weight = point_weight.normalized();
    float res = point_weight.dot(values);
    return res;
}

Interpolator &Interpolator::operator=(const Interpolator &other) {
    if (this == &other) {
        return *this;
    }
    weights = other.weights;
    points = other.points;
    values = other.values;
    points_rows = other.points_rows;                  // 参考点个数
    points_cols = other.points_cols;                  // 参考点维度
    normalization_params = other.normalization_params;// 保存每一行的最小值和跨度
    return *this;
}