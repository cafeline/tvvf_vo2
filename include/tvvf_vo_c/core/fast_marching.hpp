// include/tvvf_vo_c/core/fast_marching.hpp

#ifndef TVVF_VO_C_CORE_FAST_MARCHING_HPP_
#define TVVF_VO_C_CORE_FAST_MARCHING_HPP_

#include "tvvf_vo_c/core/field_types.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <queue>
#include <vector>
#include <array>

namespace tvvf_vo_c {

class FastMarching {
private:
    VectorField field_;
    static constexpr double DEFAULT_TRAVEL_COST = 1.0;
    
    // FMM用の状態
    enum CellStatus {
        FAR,      // 未処理
        NARROW,   // 処理中（ナローバンド）
        FROZEN    // 処理済み
    };
    
    struct FMMCell {
        int x, y;
        double time;  // 到達時間
        CellStatus status;
        
        FMMCell() : x(0), y(0), time(std::numeric_limits<double>::infinity()), status(FAR) {}
        
        bool operator>(const FMMCell& other) const {
            return time > other.time;
        }
    };
    
    std::vector<std::vector<FMMCell>> fmm_grid_;
    
    // FMMCell*の比較関数
    struct FMMCellCompare {
        bool operator()(const FMMCell* a, const FMMCell* b) const {
            return a->time > b->time;  // 小さい時間を優先
        }
    };
    
    std::priority_queue<FMMCell*, std::vector<FMMCell*>, FMMCellCompare> narrow_band_;
    
public:
    FastMarching();
    
    // マップを初期化
    void initializeFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& map);
    void initializeFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& map,
                                     const std::vector<double>& speed_layer);
    
    // ゴールからの距離場を計算
    void computeDistanceField(const Position& goal);
    
    // 距離場からベクトル場を生成
    void generateVectorField();
    
    // 指定位置のベクトルを取得
    std::array<double, 2> getVectorAt(const Position& pos) const;
    
    // フィールドを取得（テスト用）
    const VectorField& getField() const { return field_; }
    
    // Eikonalソルバー（テスト用パブリック関数）
    double solveEikonalWithKnownNeighbors(double T_left, double T_right, 
                                          double T_up, double T_down,
                                          double travel_cost = DEFAULT_TRAVEL_COST) const;
    
private:
    void initializeFromOccupancyGridImpl(const nav_msgs::msg::OccupancyGrid& map,
                                         const std::vector<double>* speed_layer);
    // Eikonal方程式のソルバー
    double solveEikonal(int x, int y);
    double solveQuadraticEikonal(double T_h, double T_v, double travel_cost) const;
    
    // ナローバンド管理
    void updateNeighbors(int x, int y);
    void updateSingleNeighbor(int x, int y);
    void clearNarrowBand();
    
    // 初期化ヘルパー
    void initializeFieldMetadata(const nav_msgs::msg::OccupancyGrid& map);
    void allocateGridMemory();
    void initializeCell(int x, int y, int8_t occupancy, double speed_value);
    bool initializeGoal(const Position& goal);
    
    // FMM実行
    void performFastMarching();
    
    // 勾配計算ヘルパー
    std::array<double, 2> computeGradientVector(int x, int y) const;
    double computeGradientX(int x, int y) const;
    double computeGradientY(int x, int y) const;
    std::array<double, 2> normalizeGradient(double dx, double dy) const;
    
    // ユーティリティ
    bool isValidGridPosition(int x, int y) const;
    void resetFMMGrid();
    double getTravelCostForCell(int x, int y) const;
};

}  // namespace tvvf_vo_c

#endif  // TVVF_VO_C_CORE_FAST_MARCHING_HPP_
