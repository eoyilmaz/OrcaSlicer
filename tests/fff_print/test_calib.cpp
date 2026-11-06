#include <catch2/catch_all.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/calib.hpp"
#include "libslic3r/libslic3r.h"
#include "test_helpers.hpp"

using namespace Slic3r;

namespace {

struct Move
{
    double feedrate; // mm/min
    double e_per_mm;
};

// Extruding moves under a ";TYPE:<type>" tag: only the first one after each tag, or all of them. The PA line test prints
// no object, so its fast segments are the first move after each ";TYPE:Outer wall" and the box under its numbers is
// all of ";TYPE:Bottom surface".
std::vector<Move> extrusions_of_type(const std::string &gcode, const std::string &type, bool first_only)
{
    const std::string  tag = ";TYPE:" + type;
    std::vector<Move>  out;
    std::istringstream lines(gcode);
    std::string        line;
    bool               relative = false;
    bool               armed    = false;
    double             x = 0., y = 0., e = 0., f = 0.;
    while (std::getline(lines, line)) {
        if (line.rfind(";TYPE:", 0) == 0) {
            armed = line == tag;
            continue;
        }
        if (line.rfind("M83", 0) == 0)
            relative = true;
        else if (line.rfind("M82", 0) == 0)
            relative = false;
        const bool is_move = line.rfind("G1 ", 0) == 0 || line.rfind("G0 ", 0) == 0;
        if (!is_move && line.rfind("G92 ", 0) != 0)
            continue;

        std::istringstream words(line.substr(0, line.find(';')));
        std::string        word;
        words >> word; // the command
        double nx = x, ny = y, ev = 0.;
        bool   has_e = false;
        while (words >> word) {
            const double v = std::stod(word.substr(1));
            switch (word[0]) {
            case 'X': nx = v; break;
            case 'Y': ny = v; break;
            case 'E': ev = v; has_e = true; break;
            case 'F': f = v; break;
            default: break;
            }
        }
        if (!is_move) { // G92
            if (has_e)
                e = ev;
            continue;
        }
        const double length = std::hypot(nx - x, ny - y);
        if (has_e && armed && length > 0.) {
            out.push_back({f, (relative ? ev : ev - e) / length});
            armed = !first_only;
        }
        if (has_e)
            e = relative ? e + ev : ev;
        x = nx;
        y = ny;
    }
    return out;
}

} // namespace

TEST_CASE("The PA line test prints with the speed and extrusion of the filament assigned to it", "[Calib]")
{
    // Filament 1: 1.75 mm, flow ratio 1, up to 21 mm3/s. Filament 2: 2.85 mm, flow ratio 0.9, up to 4 mm3/s, so its
    // volumetric limit caps the fast lines well below filament 1's.
    const int  filament      = GENERATE(1, 2);
    const bool print_numbers = GENERATE(false, true);
    CAPTURE(filament, print_numbers);
    DynamicPrintConfig config = Test::multifilament_config(2, {
        {"filament_self_index", "1,2"},
        {"filament_extruder_variant", "Direct Drive Standard;Direct Drive Standard"},
        {"filament_diameter", "1.75,2.85"},
        {"filament_flow_ratio", "1,0.9"},
        {"filament_max_volumetric_speed", "21,4"},
        {"print_flow_ratio", "1"},
        {"outer_wall_speed", "200"},
        {"initial_layer_print_height", "0.2"},
        {"enable_prime_tower", 0},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
    });
    Print print;
    Model model;
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{{{"extruder", std::to_string(filament)}}};
    Test::init_print({Test::cube(20)}, print, model, config, &overrides);
    Calib_Params params;
    params.mode          = CalibMode::Calib_PA_Line;
    params.start         = 0.;
    params.end           = 0.02;
    params.step          = 0.002;
    params.print_numbers = print_numbers;
    print.set_calib_params(params);
    const std::string       gcode    = Test::gcode(print);
    const std::vector<Move> segments = extrusions_of_type(gcode, "Outer wall", true);
    REQUIRE_FALSE(segments.empty());
    // The box under the numbers is drawn with the same line width and height as the PA lines.
    const std::vector<Move> box = extrusions_of_type(gcode, "Bottom surface", false);
    CHECK(box.empty() != print_numbers);

    // The test prints 1.5 nozzle-wide lines on the 0.2 mm first layer (CalibPressureAdvanceLine).
    const size_t idx          = size_t(filament - 1);
    const double mm3_per_mm   = Flow(0.4 * 1.5, 0.2, 0.4).mm3_per_mm();
    const double max_vol      = config.option<ConfigOptionFloats>("filament_max_volumetric_speed")->get_at(idx);
    const double diameter     = config.option<ConfigOptionFloats>("filament_diameter")->get_at(idx);
    const double flow_ratio   = config.option<ConfigOptionFloatsNullable>("filament_flow_ratio")->get_at(idx);
    // Fast lines run at the outer wall speed (at least 100 mm/s), capped by the filament's volumetric speed.
    const double fast_speed   = std::floor(std::min(std::max(100., 200.), max_vol / mm3_per_mm));
    const double e_per_mm     = mm3_per_mm * flow_ratio / (PI * diameter * diameter / 4.);
    for (const Move &move : segments) {
        CHECK_THAT(move.feedrate, Catch::Matchers::WithinAbs(fast_speed * 60., 1.));
        CHECK_THAT(move.e_per_mm, Catch::Matchers::WithinRel(e_per_mm, 0.005));
    }
    for (const Move &move : box)
        CHECK_THAT(move.e_per_mm, Catch::Matchers::WithinRel(e_per_mm, 0.005));
}

TEST_CASE("The flushing volume tower holds the requested volume per mm for every nozzle size", "[Calib]")
{
    const double      nozzle    = GENERATE(0.2, 0.4, 0.6, 0.8);
    const std::string generator = GENERATE("classic", "arachne");
    CAPTURE(nozzle, generator);
    // The dialog's default: 20 mm³/mm for a 0.4 mm nozzle.
    const double       volume_per_mm = 50. * nozzle;
    const double       layer_height  = nozzle / 2.;
    DynamicPrintConfig config        = Test::multifilament_config(2, {
        {"nozzle_diameter", std::to_string(nozzle)},
        {"layer_height", std::to_string(layer_height)},
        {"initial_layer_print_height", std::to_string(layer_height)},
        {"line_width", "105%"},
        {"outer_wall_line_width", "0"},
        {"wall_generator", generator},
        {"enable_prime_tower", 0},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
    });

    const Flow   flow         = flush_volume_tower_flow(config);
    const Vec2d  tower_size   = flush_volume_tower_size(volume_per_mm, flow);
    const double tower_height = 10.;
    Model        model;
    ModelObject *object = model.add_object();
    add_flush_volume_parts(*object, 1, 2, tower_size, tower_height, flow, {});
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    Print print;
    print.apply(model, config);
    print.process();

    // Every tower layer is one outer wall loop and nothing else. Leave out the layer the tower starts on, which may
    // still hold the top of the base, and the last one, which may be cut short.
    const PrintObject &print_object = *print.objects().front();
    size_t             tower_layers = 0;
    for (const Layer *layer : print_object.layers()) {
        if (layer->print_z < flush_volume_base_height + 2 * layer_height || layer->print_z > flush_volume_base_height + tower_height - layer_height)
            continue;
        double volume = 0.;
        for (const LayerRegion *region : layer->regions()) {
            volume += region->perimeters.total_volume();
            volume += region->fills.total_volume();
        }
        CAPTURE(layer->print_z);
        CHECK_THAT(volume / layer->height, Catch::Matchers::WithinRel(volume_per_mm, 0.02));
        ++tower_layers;
    }
    CHECK(tower_layers > 0);
}

TEST_CASE("The label engraved into the flushing volume base adds no filament change", "[Calib]")
{
    // The label font is a bundled resource.
    const std::string previous_resources = resources_dir();
    set_resources_dir(std::string(PROFILES_DIR) + "/..");
    const double       layer_height = 0.2;
    DynamicPrintConfig config       = Test::multifilament_config(2, {
        {"layer_height", std::to_string(layer_height)},
        {"initial_layer_print_height", std::to_string(layer_height)},
        {"enable_prime_tower", 0},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
    });
    const Flow         flow       = flush_volume_tower_flow(config);
    const Vec2d        tower_size = flush_volume_tower_size(20., flow);
    const TriangleMesh label      = flush_volume_label_mesh(flush_volume_label_text(50., 20.), flow.height());
    set_resources_dir(previous_resources);
    REQUIRE_FALSE(label.empty());
    const Vec2d base_size = flush_volume_base_size(tower_size, label);
    // The label sits in front of the tower, so the base is deeper than the tower and the label together.
    CHECK(base_size.y() > tower_size.y() + label.bounding_box().size().y());

    Model        model;
    ModelObject *object = model.add_object();
    add_flush_volume_parts(*object, 1, 2, tower_size, 5., flow, label);
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    Print print;
    print.apply(model, config);
    const std::string gcode = Test::gcode(print);

    // The label is cut into the top of the base: its last layer covers less than the whole base.
    const PrintObject &print_object = *print.objects().front();
    const Layer       *base_top     = nullptr;
    for (const Layer *layer : print_object.layers())
        if (layer->print_z < flush_volume_base_height + EPSILON)
            base_top = layer;
    REQUIRE(base_top != nullptr);
    double area = 0.;
    for (const ExPolygon &slice : base_top->lslices)
        area += slice.area();
    CHECK(unscaled(unscaled(area)) < 0.98 * base_size.x() * base_size.y());

    // The base prints in T0 and the tower in T1, with one change between them.
    std::istringstream lines(gcode);
    std::string        line;
    std::string        tool;
    int                changes = 0;
    while (std::getline(lines, line))
        if (line.size() > 1 && line[0] == 'T' && std::isdigit(static_cast<unsigned char>(line[1]))) {
            const std::string next = line.substr(0, line.find_first_of(" ;"));
            if (!tool.empty() && next != tool)
                ++changes;
            tool = next;
        }
    CHECK(changes == 1);
}
