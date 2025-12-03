// include/tvvf_vo_c/core/field_types.hpp

#ifndef TVVF_VO_C_CORE_FIELD_TYPES_HPP_
#define TVVF_VO_C_CORE_FIELD_TYPES_HPP_

#include <vector>
#include <array>
#include <limits>
#include <optional>
#include "tvvf_vo_c/core/types.hpp"  // Position, Velocity, DynamicObstacleをtypes.hppから使用
#include <cstddef>
#include <algorithm>

namespace tvvf_vo_c {

// グリッドマップ上の単一セル
struct GridCell {
    int x, y;                    // グリッド座標
    double distance;             // ゴールまでの距離（または到達時間）
    double potential;            // ポテンシャル値
    double speed;                // 速度場F(x)
    double clearance;            // 障害物からの距離
    bool is_obstacle;            // 障害物フラグ
    bool is_visited;             // 訪問済みフラグ
    
    GridCell() : x(0), y(0), distance(std::numeric_limits<double>::infinity()), 
                 potential(0.0), speed(1.0), clearance(0.0),
                 is_obstacle(false), is_visited(false) {}
};

// 部分領域情報
struct FieldRegion {
    Position min;
    Position max;

    FieldRegion()
        : min(Position(0.0, 0.0)), max(Position(0.0, 0.0)) {}

    FieldRegion(const Position& min_pos, const Position& max_pos)
        : min(min_pos), max(max_pos) {}

    bool is_valid() const {
        return max.x >= min.x && max.y >= min.y;
    }

    bool contains(const Position& pos) const {
        if (!is_valid()) {
            return false;
        }
        return pos.x >= min.x && pos.x <= max.x &&
               pos.y >= min.y && pos.y <= max.y;
    }
};

// 1次元ストレージへの軽量ビュー
struct GridRowView {
    GridCell* data{nullptr};
    GridCell& operator[](size_t idx) { return data[idx]; }
    const GridCell& operator[](size_t idx) const { return data[idx]; }
};

struct VectorRowView {
    std::array<double, 2>* data{nullptr};
    std::array<double, 2>& operator[](size_t idx) { return data[idx]; }
    const std::array<double, 2>& operator[](size_t idx) const { return data[idx]; }
};

// ベクトル場全体
class VectorField {
public:
    int width, height;           // グリッドサイズ
    double resolution;           // セル解像度[m]
    Position origin;             // 原点位置
    std::optional<FieldRegion> region;  // 計算対象領域
    std::vector<GridRowView> grid;  // 2Dビュー（実体は1次元ストレージ）
    std::vector<VectorRowView> vectors;  // ベクトル場ビュー
    std::vector<VectorField> overlays;   // 高解像度オーバーレイ

    VectorField();
    VectorField(const VectorField& other);
    VectorField(VectorField&& other) noexcept;
    VectorField& operator=(const VectorField& other);
    VectorField& operator=(VectorField&& other) noexcept;
    
    // グリッド座標⇔ワールド座標変換
    GridCell& at(int x, int y) {
        return grid[static_cast<size_t>(y)][static_cast<size_t>(x)];
    }
    
    const GridCell& at(int x, int y) const {
        return grid[static_cast<size_t>(y)][static_cast<size_t>(x)];
    }
    
    Position gridToWorld(int x, int y) const {
        return Position(
            origin.x + (static_cast<double>(x) + 0.5) * resolution,
            origin.y + (static_cast<double>(y) + 0.5) * resolution
        );
    }
    
    std::pair<int, int> worldToGrid(const Position& pos) const {
        int x = static_cast<int>((pos.x - origin.x) / resolution);
        int y = static_cast<int>((pos.y - origin.y) / resolution);
        return {x, y};
    }
    
    std::array<double, 2> getVector(const Position& pos) const {
        const VectorField* selected = nullptr;
        double best_resolution = std::numeric_limits<double>::infinity();

        auto select_if_contains = [&](const VectorField& candidate) {
            FieldRegion candidate_region;
            if (candidate.region.has_value()) {
                candidate_region = candidate.region.value();
            } else {
                candidate_region = FieldRegion(
                    Position(candidate.origin.x,
                             candidate.origin.y),
                    Position(
                        candidate.origin.x +
                            static_cast<double>(candidate.width) * candidate.resolution,
                        candidate.origin.y +
                            static_cast<double>(candidate.height) * candidate.resolution));
            }
            if (!candidate_region.contains(pos)) {
                return;
            }
            if (candidate.resolution < best_resolution) {
                selected = &candidate;
                best_resolution = candidate.resolution;
            }
        };

        for (const auto& overlay : overlays) {
            select_if_contains(overlay);
        }
        select_if_contains(*this);

        if (!selected) {
            return {0.0, 0.0};
        }

        auto [x, y] = selected->worldToGrid(pos);
        if (x >= 0 && x < selected->width && y >= 0 && y < selected->height) {
            return selected->vectors[static_cast<size_t>(y)][static_cast<size_t>(x)];
        }
        return {0.0, 0.0};
    }

    void resize(int new_width, int new_height);
    void set_origin(const Position& new_origin) { origin = new_origin; }
    void addOverlay(const VectorField& overlay) { overlays.push_back(overlay); }
    void clearOverlays() { overlays.clear(); }

private:
    std::vector<GridCell> grid_storage_;
    std::vector<std::array<double, 2>> vector_storage_;

    void rebuild_row_views();
};

inline VectorField::VectorField()
    : width(0), height(0), resolution(1.0), origin(0.0, 0.0) {}

inline VectorField::VectorField(const VectorField& other)
    : width(other.width),
      height(other.height),
      resolution(other.resolution),
      origin(other.origin),
      region(other.region),
      overlays(other.overlays),
      grid_storage_(other.grid_storage_),
      vector_storage_(other.vector_storage_) {
    rebuild_row_views();
}

inline VectorField::VectorField(VectorField&& other) noexcept
    : width(other.width),
      height(other.height),
      resolution(other.resolution),
      origin(other.origin),
      region(std::move(other.region)),
      overlays(std::move(other.overlays)),
      grid_storage_(std::move(other.grid_storage_)),
      vector_storage_(std::move(other.vector_storage_)) {
    rebuild_row_views();
}

inline VectorField& VectorField::operator=(const VectorField& other) {
    if (this != &other) {
        width = other.width;
        height = other.height;
        resolution = other.resolution;
        origin = other.origin;
        region = other.region;
        overlays = other.overlays;
        grid_storage_ = other.grid_storage_;
        vector_storage_ = other.vector_storage_;
        rebuild_row_views();
    }
    return *this;
}

inline VectorField& VectorField::operator=(VectorField&& other) noexcept {
    if (this != &other) {
        width = other.width;
        height = other.height;
        resolution = other.resolution;
        origin = other.origin;
        region = std::move(other.region);
        overlays = std::move(other.overlays);
        grid_storage_ = std::move(other.grid_storage_);
        vector_storage_ = std::move(other.vector_storage_);
        rebuild_row_views();
    }
    return *this;
}

inline void VectorField::resize(int new_width, int new_height) {
    width = new_width;
    height = new_height;
    const size_t cell_count =
        static_cast<size_t>(std::max(new_width, 0)) *
        static_cast<size_t>(std::max(new_height, 0));
    grid_storage_.resize(cell_count);
    vector_storage_.resize(cell_count);
    rebuild_row_views();
}

inline void VectorField::rebuild_row_views() {
    grid.resize(static_cast<size_t>(std::max(height, 0)));
    vectors.resize(static_cast<size_t>(std::max(height, 0)));

    if (width <= 0 || height <= 0 || grid_storage_.empty() || vector_storage_.empty()) {
        for (auto& row : grid) {
            row.data = nullptr;
        }
        for (auto& row : vectors) {
            row.data = nullptr;
        }
        return;
    }

    GridCell* grid_ptr = grid_storage_.data();
    auto* vector_ptr = vector_storage_.data();
    for (int y = 0; y < height; ++y) {
        const size_t offset = static_cast<size_t>(y) * static_cast<size_t>(width);
        grid[static_cast<size_t>(y)].data = grid_ptr + offset;
        vectors[static_cast<size_t>(y)].data = vector_ptr + offset;
    }
}

}  // namespace tvvf_vo_c

#endif  // TVVF_VO_C_CORE_FIELD_TYPES_HPP_
