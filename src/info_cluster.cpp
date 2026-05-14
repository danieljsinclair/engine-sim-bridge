#include "../include/info_cluster.h"
#include "../include/engine_sim_application.h"
#include <string>

InfoCluster::InfoCluster() : m_engine(nullptr), m_logMessage("Started") {
}

InfoCluster::~InfoCluster() {
    /* void */
}

void InfoCluster::initialize(EngineSimApplication *app) {
    UiElement::initialize(app);
}

void InfoCluster::destroy() {
    UiElement::destroy();
}

void InfoCluster::update(float dt) {
    UiElement::update(dt);
}

void InfoCluster::render() {
    auto* app = static_cast<EngineSimApplication*>(UiElement::getApplication());
    if (!app) return;

#ifdef ATG_ENGINE_SIM_PRESET_MODE
    // GREEN: Loading an actual .mr script via Piranha (existing `--script` behavior on `engine-sim-cli`)
    // CYAN: Loading a pre-compiled preset via PresetRegistry (new `--script` behavior on `engine-sim-cli-baked`)
#else
    // RED: Loading a pre-compiled preset via PresetRegistry (new `--script` behavior on `engine-sim-cli-baked`)
#endif

    const char* modeName =
#ifdef ATG_ENGINE_SIM_PRESET_MODE
        "PRESET MODE"
#else
        "PIRANHA SCRIPT MODE"
#endif
    ;

    app->drawAlignedText(
        "ENGINE SIMULATOR",
        Grid::get(m_bounds, 0).inset(10.0f).move({ 0.0f, -21.0f }),
        42.0f,
        Bounds::bl,
        Bounds::bl);
    app->drawAlignedText(
        modeName,
        titleSplit.get(titleBounds, 0).inset(10.0f).move({ 0.0f, 5.0f }),
        24.0f,
        Bounds::tl,
        Bounds::tl);
}
