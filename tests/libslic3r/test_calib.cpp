#include <catch2/catch_all.hpp>

#include <algorithm>
#include "libslic3r/CustomGCode.hpp"
#include <cmath>
#include <cstddef>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/calib.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"

using namespace Slic3r;

namespace {

// The width-resolution getters are protected; expose them so the resolution can be asserted directly.
struct PaPatternProbe : public CalibPressureAdvancePattern
{
    using CalibPressureAdvancePattern::CalibPressureAdvancePattern;
    using CalibPressureAdvancePattern::line_width;
    using CalibPressureAdvancePattern::line_width_first_layer;
};

} // namespace

TEST_CASE("Zero calibration line width resolves to a positive default", "[Calib][Regression]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"line_width", "0"},
        {"initial_layer_line_width", "0"},
    });

    Model model;
    model.add_object("cube", "", make_cube(20, 20, 20))->add_instance();

    Calib_Params params;
    params.mode = CalibMode::Calib_PA_Pattern;

    PaPatternProbe pattern(params, config, /* is_bbl_machine */ true, *model.objects.front(), Vec3d(0, 0, 0));

    REQUIRE(pattern.line_width() > 0.);
    REQUIRE(pattern.line_width_first_layer() > 0.);
}

namespace {

struct EndState { double final_e; double max_e; };

EndState simulate_absolute_e(const std::string &gcode)
{
    double final_e = 0.;
    double max_e   = 0.;

    std::istringstream lines(gcode);
    std::string        line;
    while (std::getline(lines, line)) {
        std::istringstream words(line);
        std::string        op;
        if (!(words >> op))
            continue;
        if (op != "G1" && op != "G0" && op != "G92")
            continue;

        std::string word;
        while (words >> word) {
            if (word.size() >= 2 && word[0] == 'E') {
                final_e = std::stod(word.substr(1));
                max_e   = std::max(max_e, final_e);
                break;
            }
        }
    }

    return {final_e, max_e};
}

} // namespace

TEST_CASE("PA pattern resets the extruder after the final layer in absolute E mode", "[Calib][Regression]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"use_relative_e_distances", "0"},
        {"line_width", "0.45"},
        {"initial_layer_line_width", "0.45"},
    });

    Model model;
    model.add_object("cube", "", make_cube(20, 20, 20))->add_instance();

    Calib_Params params;
    params.mode  = CalibMode::Calib_PA_Pattern;
    params.start = 0.;
    params.end   = 0.08;
    params.step  = 0.002;

    CalibPressureAdvancePattern pattern(params, config, /* is_bbl_machine */ false, *model.objects.front(), Vec3d(0, 0, 0));
    const CustomGCode::Info     info = pattern.generate_custom_gcodes(config, /* is_bbl_machine */ false, *model.objects.front(),
                                                                      Vec3d(0, 0, 0));

    std::string gcode;
    for (const CustomGCode::Item &item : info.gcodes)
        gcode += item.extra;

    const EndState state = simulate_absolute_e(gcode);

    REQUIRE(state.max_e > 1.);
    REQUIRE_THAT(state.final_e, Catch::Matchers::WithinAbs(0., 1e-9));
}

TEST_CASE("Flush volume change sequence covers every pair as one chain", "[Calib]")
{
    std::vector<std::vector<int>> id_sets{{1, 3, 6}};
    for (int n = 2; n <= 8; ++n) {
        std::vector<int> ids(n);
        std::iota(ids.begin(), ids.end(), 1);
        id_sets.push_back(ids);
    }

    for (const std::vector<int> &ids : id_sets) {
        const size_t n = ids.size();
        CAPTURE(n);
        const std::vector<std::pair<int, int>> changes = flush_volume_change_sequence(ids);

        REQUIRE(changes.size() == n * (n - 1));
        CHECK(changes.front().first == ids.front());

        std::set<std::pair<int, int>> unique_changes(changes.begin(), changes.end());
        CHECK(unique_changes.size() == changes.size());
        for (const auto &[from, to] : changes) {
            CHECK(from != to);
            CHECK(std::find(ids.begin(), ids.end(), from) != ids.end());
            CHECK(std::find(ids.begin(), ids.end(), to) != ids.end());
        }
        for (size_t i = 1; i < changes.size(); ++i)
            CHECK(changes[i - 1].second == changes[i].first);
    }

    CHECK(flush_volume_change_sequence({}).empty());
    CHECK(flush_volume_change_sequence({2}).empty());
}

TEST_CASE("Flush volume grid fills the bed row by row and avoids excluded areas", "[Calib]")
{
    const Vec2d footprint(18.8, 11.);
    const Vec2d pitch = footprint + Vec2d::Constant(41.);
    const auto  rectangle = [](double x0, double y0, double x1, double y1) { return Pointfs{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}; };

    SECTION("rectangular bed")
    {
        const std::vector<Vec2d> cells = flush_volume_grid_cells(rectangle(0., 0., 235., 235.), {}, footprint, pitch, 5.);
        REQUIRE(cells.size() == 4 * 5);
        CHECK(cells.front().x() < cells[1].x());
        CHECK(cells.front().y() == cells[1].y());
        CHECK(cells[3].y() < cells[4].y());
        Vec2d sum = Vec2d::Zero();
        for (const Vec2d &cell : cells) {
            CHECK(cell.x() - footprint.x() / 2 >= 0.);
            CHECK(cell.y() - footprint.y() / 2 >= 0.);
            CHECK(cell.x() + footprint.x() / 2 <= 235.);
            CHECK(cell.y() + footprint.y() / 2 <= 235.);
            sum += cell;
        }
        // The grid is centred on the bed.
        CHECK((sum / double(cells.size()) - Vec2d(117.5, 117.5)).norm() < 1e-6);
    }

    SECTION("bed with its origin in the centre")
    {
        const std::vector<Vec2d> cells = flush_volume_grid_cells(rectangle(-117.5, -117.5, 117.5, 117.5), {}, footprint, pitch, 5.);
        REQUIRE(cells.size() == 4 * 5);
        CHECK(cells.front().x() < 0.);
        CHECK(cells.front().y() < 0.);
    }

    SECTION("excluded area in the front left corner")
    {
        const Polygons excluded{Polygon::new_scale(rectangle(0., 0., 60., 40.))};
        const std::vector<Vec2d> all   = flush_volume_grid_cells(rectangle(0., 0., 235., 235.), {}, footprint, pitch, 5.);
        const std::vector<Vec2d> cells = flush_volume_grid_cells(rectangle(0., 0., 235., 235.), excluded, footprint, pitch, 5.);
        REQUIRE(cells.size() == all.size() - 1);
        CHECK(cells.front() == all[1]);
    }

    SECTION("round bed")
    {
        Pointfs circle;
        for (int i = 0; i < 64; ++i)
            circle.emplace_back(100. * std::cos(2. * M_PI * i / 64), 100. * std::sin(2. * M_PI * i / 64));
        const std::vector<Vec2d> cells = flush_volume_grid_cells(circle, {}, footprint, pitch, 5.);
        REQUIRE(!cells.empty());
        // The corners of the square grid fall outside the circle.
        CHECK(cells.size() < 3 * 4);
        for (const Vec2d &cell : cells)
            CHECK((cell.cwiseAbs() + footprint / 2).norm() <= 100.);
    }

    SECTION("bed smaller than one cell")
    {
        CHECK(flush_volume_grid_cells(rectangle(0., 0., 20., 20.), {}, footprint, pitch, 5.).empty());
    }
}
