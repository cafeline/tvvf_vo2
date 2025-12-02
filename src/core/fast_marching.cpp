// src/core/fast_marching.cpp

#include "tvvf_vo_c/core/fast_marching.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace tvvf_vo_c {

// 定数定義
namespace {
    constexpr int8_t UNKNOWN_CELL = -1;            // 不明なセルの値
    constexpr double GRID_COST = 1.0;              // グリッド単位での移動コスト
    constexpr double GRADIENT_EPSILON = 1e-6;      // 勾配計算の最小値
    
    // 4近傍の方向ベクトル（FMM用）
    constexpr int DX[4] = {-1, 1, 0, 0};
    constexpr int DY[4] = {0, 0, -1, 1};
}

FastMarching::FastMarching() {}

void FastMarching::initializeFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& map) {
    initializeFromOccupancyGridImpl(map, nullptr);
}

void FastMarching::initializeFromOccupancyGrid(
    const nav_msgs::msg::OccupancyGrid& map,
    const std::vector<double>& speed_layer) {
    initializeFromOccupancyGridImpl(map, &speed_layer);
}

void FastMarching::setOccupancyThresholds(double occupied_threshold, double free_threshold) {
    occupied_threshold_ = occupied_threshold;
    free_threshold_ = free_threshold;
}

void FastMarching::initializeFromOccupancyGridImpl(
    const nav_msgs::msg::OccupancyGrid& map,
    const std::vector<double>* speed_layer) {
    // フィールドメタデータの設定
    initializeFieldMetadata(map);
    
    // グリッドメモリの確保と初期化
    allocateGridMemory();
    
    // 各セルの初期化
    for (int y = 0; y < field_.height; ++y) {
        for (int x = 0; x < field_.width; ++x) {
            const size_t index = static_cast<size_t>(y) * field_.width + x;
            double speed_value = 1.0;
            if (speed_layer && index < speed_layer->size()) {
                speed_value = (*speed_layer)[index];
            }
            initializeCell(x, y, map.data[index], speed_value);
        }
    }
}

void FastMarching::initializeFieldMetadata(const nav_msgs::msg::OccupancyGrid& map) {
    field_.width = map.info.width;
    field_.height = map.info.height;
    field_.resolution = map.info.resolution;
    field_.origin = Position(map.info.origin.position.x, 
                            map.info.origin.position.y);
}

void FastMarching::allocateGridMemory() {
    field_.grid.resize(field_.height);
    field_.vectors.resize(field_.height);
    fmm_grid_.resize(field_.height);
    
    for (int y = 0; y < field_.height; ++y) {
        field_.grid[y].resize(field_.width);
        field_.vectors[y].resize(field_.width, {0.0, 0.0});
        fmm_grid_[y].resize(field_.width);
    }
}

void FastMarching::initializeCell(int x, int y, int8_t occupancy, double speed_value) {
    GridCell& cell = field_.grid[y][x];
    FMMCell& fmm_cell = fmm_grid_[y][x];
    
    // グリッド座標を設定
    cell.x = x;
    cell.y = y;
    fmm_cell.x = x;
    fmm_cell.y = y;
    
    // 障害物判定（パラメータ閾値で判定）
    bool occupancy_obstacle = false;
    if (occupancy == UNKNOWN_CELL) {
        occupancy_obstacle = true;
    } else {
        const double occ = static_cast<double>(occupancy);
        occupancy_obstacle = (occ >= occupied_threshold_) ||
            (occ > free_threshold_ && occ < occupied_threshold_);
    }
    if (!std::isfinite(speed_value)) {
        speed_value = 0.0;
    }
    cell.speed = std::max(0.0, speed_value);
    cell.is_obstacle = occupancy_obstacle || (cell.speed <= 0.0);
    cell.distance = std::numeric_limits<double>::infinity();
    cell.is_visited = false;
    
    // FMMセルの初期化
    fmm_cell.time = std::numeric_limits<double>::infinity();
    fmm_cell.status = FAR;
}

void FastMarching::computeDistanceField(const Position& goal) {
    // FMMグリッドとナローバンドをリセット
    resetFMMGrid();
    clearNarrowBand();
    
    // ゴール位置の初期化
    if (!initializeGoal(goal)) {
        return;  // ゴールが無効な場合は終了
    }
    
    // Fast Marching Method メインループ
    performFastMarching();
}

void FastMarching::clearNarrowBand() {
    while (!narrow_band_.empty()) {
        narrow_band_.pop();
    }
}

bool FastMarching::initializeGoal(const Position& goal) {
    const auto [gx, gy] = field_.worldToGrid(goal);
    
    if (!isValidGridPosition(gx, gy) || field_.grid[gy][gx].is_obstacle) {
        return false;
    }
    
    // ゴールセルを初期化
    fmm_grid_[gy][gx].time = 0;
    fmm_grid_[gy][gx].status = FROZEN;
    field_.grid[gy][gx].distance = 0;
    
    // ゴール周辺をナローバンドに追加
    updateNeighbors(gx, gy);
    
    return true;
}

void FastMarching::performFastMarching() {
    while (!narrow_band_.empty()) {
        // 最小時間のセルを取得
        FMMCell* current = narrow_band_.top();
        narrow_band_.pop();
        
        if (current->status != NARROW) continue;
        
        // FROZEN状態に更新
        current->status = FROZEN;
        field_.grid[current->y][current->x].distance = current->time * field_.resolution;
        
        // 隣接セル更新
        updateNeighbors(current->x, current->y);
    }
}

void FastMarching::generateVectorField() {
    for (int y = 0; y < field_.height; ++y) {
        for (int x = 0; x < field_.width; ++x) {
            field_.vectors[y][x] = computeGradientVector(x, y);
        }
    }
}

std::array<double, 2> FastMarching::computeGradientVector(int x, int y) const {
    const GridCell& cell = field_.grid[y][x];
    
    // 障害物または到達不可能なセルはゼロベクトル
    if (cell.is_obstacle || std::isinf(cell.distance)) {
        return {0.0, 0.0};
    }
    
    // 中心差分による勾配計算
    const double dx = computeGradientX(x, y);
    const double dy = computeGradientY(x, y);
    
    // 勾配ベクトルの正規化
    return normalizeGradient(dx, dy);
}

double FastMarching::computeGradientX(int x, int y) const {
    if (x <= 0 || x >= field_.width - 1) {
        return 0.0;
    }
    
    const double dist_left = field_.grid[y][x-1].distance;
    const double dist_right = field_.grid[y][x+1].distance;
    
    if (std::isinf(dist_left) || std::isinf(dist_right)) {
        return 0.0;
    }
    
    return (dist_left - dist_right) / (2.0 * field_.resolution);
}

double FastMarching::computeGradientY(int x, int y) const {
    if (y <= 0 || y >= field_.height - 1) {
        return 0.0;
    }
    
    const double dist_up = field_.grid[y-1][x].distance;
    const double dist_down = field_.grid[y+1][x].distance;
    
    if (std::isinf(dist_up) || std::isinf(dist_down)) {
        return 0.0;
    }
    
    return (dist_up - dist_down) / (2.0 * field_.resolution);
}

std::array<double, 2> FastMarching::normalizeGradient(double dx, double dy) const {
    const double norm = std::sqrt(dx*dx + dy*dy);
    
    if (norm > GRADIENT_EPSILON) {
        return {dx / norm, dy / norm};
    }
    
    return {0.0, 0.0};
}

std::array<double, 2> FastMarching::getVectorAt(const Position& pos) const {
    return field_.getVector(pos);
}

double FastMarching::solveEikonalWithKnownNeighbors(double T_left, double T_right, 
                                                    double T_up, double T_down,
                                                    double travel_cost) const {
    // 水平・垂直方向の最小値を取得
    const double T_h = std::min(T_left, T_right);
    const double T_v = std::min(T_up, T_down);
    
    // 両方向とも無限大の場合
    if (std::isinf(T_h) && std::isinf(T_v)) {
        return std::numeric_limits<double>::infinity();
    }
    
    // 片方向のみ有効な場合
    if (std::isinf(T_h)) {
        return T_v + travel_cost;
    }
    if (std::isinf(T_v)) {
        return T_h + travel_cost;
    }
    
    // 両方向が有効な場合：二次方程式を解く
    return solveQuadraticEikonal(T_h, T_v, travel_cost);
}

double FastMarching::solveQuadraticEikonal(double T_h, double T_v,
                                           double travel_cost) const {
    const double a = T_h - T_v;
    const double discriminant = 2 * travel_cost * travel_cost - a * a;
    
    if (discriminant < 0) {
        return std::min(T_h, T_v) + travel_cost;
    }
    
    return (T_h + T_v + std::sqrt(discriminant)) / 2.0;
}

double FastMarching::solveEikonal(int x, int y) {
    // 隣接セルの到達時間を取得
    double T_left = std::numeric_limits<double>::infinity();
    double T_right = std::numeric_limits<double>::infinity();
    double T_up = std::numeric_limits<double>::infinity();
    double T_down = std::numeric_limits<double>::infinity();
    
    if (x > 0 && fmm_grid_[y][x-1].status == FROZEN)
        T_left = fmm_grid_[y][x-1].time;
    if (x < field_.width-1 && fmm_grid_[y][x+1].status == FROZEN)
        T_right = fmm_grid_[y][x+1].time;
    if (y > 0 && fmm_grid_[y-1][x].status == FROZEN)
        T_up = fmm_grid_[y-1][x].time;
    if (y < field_.height-1 && fmm_grid_[y+1][x].status == FROZEN)
        T_down = fmm_grid_[y+1][x].time;
    
    const double travel_cost = getTravelCostForCell(x, y);
    return solveEikonalWithKnownNeighbors(T_left, T_right, T_up, T_down, travel_cost);
}

void FastMarching::updateNeighbors(int x, int y) {
    for (int i = 0; i < 4; ++i) {
        const int nx = x + DX[i];
        const int ny = y + DY[i];
        
        if (!isValidGridPosition(nx, ny)) continue;
        
        updateSingleNeighbor(nx, ny);
    }
}

void FastMarching::updateSingleNeighbor(int x, int y) {
    FMMCell& neighbor = fmm_grid_[y][x];
    
    // 障害物または処理済みセルはスキップ
    if (field_.grid[y][x].is_obstacle || neighbor.status == FROZEN) {
        return;
    }
    
    // Eikonal方程式を解いて時間更新
    const double new_time = solveEikonal(x, y);
    
    if (new_time < neighbor.time) {
        neighbor.time = new_time;
        
        if (neighbor.status == FAR) {
            neighbor.status = NARROW;
            narrow_band_.push(&neighbor);
        }
    }
}

bool FastMarching::isValidGridPosition(int x, int y) const {
    return x >= 0 && x < field_.width && y >= 0 && y < field_.height;
}

void FastMarching::resetFMMGrid() {
    for (auto& row : fmm_grid_) {
        for (auto& cell : row) {
            cell.time = std::numeric_limits<double>::infinity();
            cell.status = FAR;
        }
    }
}

double FastMarching::getTravelCostForCell(int x, int y) const {
    const double speed = field_.grid[y][x].speed;
    if (!std::isfinite(speed) || speed <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    return GRID_COST / speed;
}

}  // namespace tvvf_vo_c
