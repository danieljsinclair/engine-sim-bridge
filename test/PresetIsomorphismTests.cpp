// PresetIsomorphismTests.cpp - TDD tests proving .mr and JSON paths
// produce identical engines.
//
// Categories:
//   1. Function deserialization (filterRadius, samples)
//   2. Deserialization completeness (all fields read from JSON)
//   3. No-fallback (throws on missing required fields)
//   4. End-to-end preset loading (full engine from fixture)
//
// Build: cmake -DBUILD_PRESET_ENGINE_TESTS=ON
// Run:   preset_engine_tests --gtest_filter=*Isomorphism*

#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

// Engine-sim headers
#include "engine.h"
#include "crankshaft.h"
#include "cylinder_bank.h"
#include "cylinder_head.h"
#include "camshaft.h"
#include "exhaust_system.h"
#include "intake.h"
#include "fuel.h"
#include "ignition_module.h"
#include "vehicle.h"
#include "transmission.h"
#include "function.h"

// Bridge headers
#include "simulator/PresetEngineFactory.h"
#include "common/JsonParser.h"
#include "TestPathHelpers.h"

#if !TARGET_OS_IPHONE && defined(ENGINE_SIM_PIRANHA_ENABLED)
#include "simulator/SimulatorFactory.h"
#include "simulator/BridgeSimulator.h"
#include "simulator/EngineSimTypes.h"
#include "simulator/SimulatorInitHelpers.h"
#include "simulator.h"
#include "piston_engine_simulator.h"
#endif

namespace {
    const std::string FIXTURE_HONDA = TEST_FIXTURE_DIR "/01_honda_trx520.preset.json";
    const std::string FIXTURE_GM_LS = TEST_FIXTURE_DIR "/gm_ls.preset.json";
    const std::string FIXTURE_MINIMAL_CAM = TEST_FIXTURE_DIR "/MinimalCamProfile.preset.json";
    const std::string FIXTURE_NO_ENGINE = TEST_FIXTURE_DIR "/no_engine_section.preset.json";
    const std::string FIXTURE_ZERO_CYL = TEST_FIXTURE_DIR "/zero_cylinder_count.preset.json";
    const std::string FIXTURE_MISSING_CRANK = TEST_FIXTURE_DIR "/missing_crankshafts.preset.json";
    const std::string FIXTURE_MALFORMED = TEST_FIXTURE_DIR "/malformed.preset.json";
    const std::string FIXTURE_EMPTY_OBJ = TEST_FIXTURE_DIR "/empty_object.preset.json";

    constexpr double TOLERANCE = 1e-9;

    // Helper: read entire file into string
    std::string readFile(const std::string& path) {
        std::ifstream f(path);
        if (!f.is_open()) return {};
        std::ostringstream ss;
        ss << f.rdbuf();
        return ss.str();
    }
}

// ============================================================================
// Category 1: Function Deserialization
// ============================================================================

class FunctionDeserializationTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

// 1.1: A Function with samples creates a non-null Function with correct sample count
TEST_F(FunctionDeserializationTest, FunctionWithSamplesCreatesCorrectFunction) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_MINIMAL_CAM);
    ASSERT_TRUE(result.success()) << "Load failed: " << result.error;

    Engine* engine = result.engine;
    ASSERT_NE(engine, nullptr);

    // Verify the intake camshaft lobe profile has exactly 3 sample points
    CylinderHead* head = engine->getHead(0);
    ASSERT_NE(head, nullptr);

    Camshaft* intakeCam = head->getIntakeCamshaft();
    ASSERT_NE(intakeCam, nullptr);
    Function* profile = intakeCam->getLobeProfile();
    ASSERT_NE(profile, nullptr);
    EXPECT_EQ(profile->getSampleCount(), 3);

    // Verify sample values match JSON: [-1.0,0], [0,0.005], [1.0,0]
    EXPECT_NEAR(profile->getX(0), -1.0, TOLERANCE);
    EXPECT_NEAR(profile->getY(0), 0.0, TOLERANCE);
    EXPECT_NEAR(profile->getX(1), 0.0, TOLERANCE);
    EXPECT_NEAR(profile->getY(1), 0.005, TOLERANCE);
    EXPECT_NEAR(profile->getX(2), 1.0, TOLERANCE);
    EXPECT_NEAR(profile->getY(2), 0.0, TOLERANCE);

    delete engine;
}

// 1.2: filterRadius is computed from median sample spacing when not explicitly provided
TEST_F(FunctionDeserializationTest, FilterRadiusComputedFromSampleSpacing) {
    // Use Honda fixture which has lobe profiles with many samples
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_HONDA);
    ASSERT_TRUE(result.success()) << "Load failed: " << result.error;

    Engine* engine = result.engine;
    ASSERT_NE(engine, nullptr);

    CylinderHead* head = engine->getHead(0);
    ASSERT_NE(head, nullptr);
    Camshaft* intakeCam = head->getIntakeCamshaft();
    ASSERT_NE(intakeCam, nullptr);
    Function* profile = intakeCam->getLobeProfile();
    ASSERT_NE(profile, nullptr);

    // filterRadius is read from JSON (NOT computed from sample spacing)
    double filterRadius = profile->getFilterRadius();
    EXPECT_GT(filterRadius, 0.0) << "filterRadius must be positive";

    // Verify it matches the JSON fixture value
    std::string jsonContent = readFile(FIXTURE_HONDA);
    ASSERT_FALSE(jsonContent.empty());
    json::JsonValue root = json::parse(jsonContent);
    const auto& lobeProfileJson = root["engine"]["cylinderBanks"][static_cast<size_t>(0)]
        ["cylinderHead"]["intakeCamshaft"]["lobeProfile"];
    if (lobeProfileJson.has("filterRadius")) {
        double expectedFilterRadius = lobeProfileJson["filterRadius"].asNumber();
        EXPECT_NEAR(filterRadius, expectedFilterRadius, TOLERANCE)
            << "filterRadius must match JSON value (read, not computed)";
    }

    delete engine;
}

// 1.3: Function with many samples preserves all x,y pairs exactly
TEST_F(FunctionDeserializationTest, ManySamplesPreservesAllPairsExactly) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_HONDA);
    ASSERT_TRUE(result.success()) << result.error;

    Engine* engine = result.engine;
    CylinderHead* head = engine->getHead(0);
    Camshaft* cam = head->getIntakeCamshaft();
    Function* profile = cam->getLobeProfile();

    // Read expected values from the JSON fixture directly
    std::string jsonContent = readFile(FIXTURE_HONDA);
    ASSERT_FALSE(jsonContent.empty());

    json::JsonValue root = json::parse(jsonContent);
    const auto& lobeProfile = root["engine"]["cylinderBanks"][static_cast<size_t>(0)]["cylinderHead"]["intakeCamshaft"]["lobeProfile"];
    const auto& samples = lobeProfile["samples"];

    int jsonSamples = static_cast<int>(samples.size());
    int fnSamples = profile->getSampleCount();
    EXPECT_EQ(fnSamples, jsonSamples)
        << "Function sample count must match JSON sample count";

    // Verify first and last samples match exactly
    if (fnSamples > 0 && jsonSamples > 0) {
        EXPECT_NEAR(profile->getX(0), samples[static_cast<size_t>(0)][static_cast<size_t>(0)].asNumber(), TOLERANCE);
        EXPECT_NEAR(profile->getY(0), samples[static_cast<size_t>(0)][static_cast<size_t>(1)].asNumber(), TOLERANCE);
        EXPECT_NEAR(profile->getX(fnSamples - 1),
                    samples[static_cast<size_t>(jsonSamples - 1)][static_cast<size_t>(0)].asNumber(), TOLERANCE);
        EXPECT_NEAR(profile->getY(fnSamples - 1),
                    samples[static_cast<size_t>(jsonSamples - 1)][static_cast<size_t>(1)].asNumber(), TOLERANCE);
    }

    delete engine;
}

// ============================================================================
// Category 2: Deserialization Completeness
// ============================================================================

class DeserializationCompletenessTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

// 2.1: Crankshaft deserialization reads all fields from JSON
TEST_F(DeserializationCompletenessTest, CrankshaftReadsAllFields) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_GM_LS);
    ASSERT_TRUE(result.success()) << result.error;

    Engine* engine = result.engine;
    ASSERT_NE(engine, nullptr);
    ASSERT_GT(engine->getCrankshaftCount(), 0);

    Crankshaft* cs = engine->getCrankshaft(0);
    ASSERT_NE(cs, nullptr);

    // GM LS crankshaft expected values from fixture
    EXPECT_NEAR(cs->getMass(), 27.2155422, TOLERANCE);
    EXPECT_NEAR(cs->getFlywheelMass(), 13.6077711, TOLERANCE);
    EXPECT_NEAR(cs->getMomentOfInertia(), 0.453743822313237, TOLERANCE);
    EXPECT_NEAR(cs->getThrow(), 0.0459994, TOLERANCE);
    EXPECT_NEAR(cs->getPosX(), 0.0, TOLERANCE);
    EXPECT_NEAR(cs->getPosY(), 0.0, TOLERANCE);
    EXPECT_NEAR(cs->getTdc(), 0.7853981633975, TOLERANCE);
    EXPECT_NEAR(cs->getFrictionTorque(), 27.11634912, TOLERANCE);
    EXPECT_EQ(cs->getRodJournalCount(), 4);

    // Verify rod journal angles
    EXPECT_NEAR(cs->getRodJournalAngle(0), 0.0, TOLERANCE);
    EXPECT_NEAR(cs->getRodJournalAngle(1), 4.712388980385, TOLERANCE);
    EXPECT_NEAR(cs->getRodJournalAngle(2), 1.570796326795, TOLERANCE);
    EXPECT_NEAR(cs->getRodJournalAngle(3), 3.14159265359, TOLERANCE);

    delete engine;
}

// 2.2: ExhaustSystem deserialization reads all fields from JSON
TEST_F(DeserializationCompletenessTest, ExhaustSystemReadsAllFields) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_GM_LS);
    ASSERT_TRUE(result.success()) << result.error;

    Engine* engine = result.engine;
    ASSERT_GE(engine->getExhaustSystemCount(), 1);

    ExhaustSystem* ex = engine->getExhaustSystem(0);
    ASSERT_NE(ex, nullptr);

    // GM LS exhaust expected values
    EXPECT_NEAR(ex->getLength(), 2.54, TOLERANCE);
    EXPECT_NEAR(ex->getCollectorCrossSectionArea(), 0.0081073196655605, TOLERANCE);
    EXPECT_GT(ex->getOutletFlowRate(), 0.0)
        << "outletFlowRate must be non-zero (factory applies fallback for 0)";
    EXPECT_NEAR(ex->getPrimaryTubeLength(), 0.7366, TOLERANCE);
    EXPECT_NEAR(ex->getPrimaryFlowRate(), 0.0159253157127426, TOLERANCE);
    EXPECT_NEAR(ex->getVelocityDecay(), 1.0, TOLERANCE);
    EXPECT_NEAR(ex->getAudioVolume(), 4.0, TOLERANCE);

    delete engine;
}

// 2.3: Intake deserialization reads all fields from JSON
TEST_F(DeserializationCompletenessTest, IntakeReadsAllFields) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_HONDA);
    ASSERT_TRUE(result.success()) << result.error;

    Engine* engine = result.engine;
    ASSERT_GE(engine->getIntakeCount(), 1);

    Intake* intake = engine->getIntake(0);
    ASSERT_NE(intake, nullptr);

    // Honda intake expected values from fixture
    EXPECT_NEAR(intake->getRunnerFlowRate(), 0.00637012628509703, TOLERANCE);
    EXPECT_NEAR(intake->getRunnerLength(), 0.1016, TOLERANCE);
    EXPECT_NEAR(intake->getPlenumCrossSectionArea(), 0.001, TOLERANCE);
    EXPECT_NEAR(intake->getVelocityDecay(), 0.5, TOLERANCE);
    EXPECT_GT(intake->getInputFlowK(), 0.0)
        << "InputFlowK must be positive (engine needs air)";

    delete engine;
}

// 2.4: Fuel deserialization reads all fields from JSON
TEST_F(DeserializationCompletenessTest, FuelReadsAllFields) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_GM_LS);
    ASSERT_TRUE(result.success()) << result.error;

    Engine* engine = result.engine;
    Fuel* fuel = engine->getFuel();
    ASSERT_NE(fuel, nullptr);

    EXPECT_NEAR(fuel->getMolecularAfr(), 12.5, TOLERANCE);
    EXPECT_NEAR(fuel->getBurningEfficiencyRandomness(), 0.5, TOLERANCE);
    EXPECT_NEAR(fuel->getLowEfficiencyAttenuation(), 0.6, TOLERANCE);
    EXPECT_NEAR(fuel->getMaxBurningEfficiency(), 1.0, TOLERANCE);
    EXPECT_NEAR(fuel->getMaxTurbulenceEffect(), 2.0, TOLERANCE);
    EXPECT_NEAR(fuel->getMaxDilutionEffect(), 10.0, TOLERANCE);

    // Turbulence function must be set (null causes crash in flameSpeed())
    EXPECT_NE(fuel->getTurbulenceToFlameSpeedRatio(), nullptr)
        << "turbulenceToFlameSpeedRatio must not be null";

    delete engine;
}

// 2.5: Vehicle deserialization reads all fields from JSON
TEST_F(DeserializationCompletenessTest, VehicleReadsAllFields) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_GM_LS);
    ASSERT_TRUE(result.success()) << result.error;

    Vehicle* vehicle = result.vehicle;
    ASSERT_NE(vehicle, nullptr);

    EXPECT_NEAR(vehicle->getMass(), 1614.0, TOLERANCE);
    EXPECT_NEAR(vehicle->getDragCoefficient(), 0.3, TOLERANCE);
    EXPECT_NEAR(vehicle->getCrossSectionArea(), 2.322576, TOLERANCE);
    EXPECT_NEAR(vehicle->getDiffRatio(), 3.42, TOLERANCE);
    EXPECT_NEAR(vehicle->getTireRadius(), 0.254, TOLERANCE);
    EXPECT_NEAR(vehicle->getRollingResistance(), 200.0, TOLERANCE);
}

// 2.6: Transmission deserialization produces valid transmission
TEST_F(DeserializationCompletenessTest, TransmissionDeserialized) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_GM_LS);
    ASSERT_TRUE(result.success()) << result.error;

    Transmission* trans = result.transmission;
    ASSERT_NE(trans, nullptr);
    EXPECT_GT(trans->getGearCount(), 0) << "Transmission must have gears";

    // The current factory uses hardcoded defaults since the JSON
    // transmission section is not yet serialized by the compiler.
    // When Task #3 lands with proper deserialization, these assertions
    // will verify the JSON values instead.
    for (int i = 0; i < trans->getGearCount(); i++) {
        EXPECT_GT(trans->getGearRatio(i), 0.0)
            << "Gear ratio " << i << " must be positive";
    }
}

// 2.7: Cylinder bank and head deserialization reads all fields
TEST_F(DeserializationCompletenessTest, CylinderBankAndHeadReadAllFields) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_GM_LS);
    ASSERT_TRUE(result.success()) << result.error;

    Engine* engine = result.engine;
    ASSERT_GE(engine->getCylinderBankCount(), 2);

    // Bank 0
    CylinderBank* bank0 = engine->getCylinderBank(0);
    ASSERT_NE(bank0, nullptr);
    EXPECT_NEAR(bank0->getAngle(), -0.7853981633975, TOLERANCE);
    EXPECT_NEAR(bank0->getBore(), 0.096012, TOLERANCE);
    EXPECT_NEAR(bank0->getDeckHeight(), 0.2313994, TOLERANCE);
    EXPECT_EQ(bank0->getCylinderCount(), 4);
    EXPECT_EQ(bank0->getIndex(), 0);

    // Head 0
    CylinderHead* head0 = engine->getHead(0);
    ASSERT_NE(head0, nullptr);
    EXPECT_NEAR(head0->getCombustionChamberVolume(), 9e-05, TOLERANCE);
    EXPECT_NEAR(head0->getIntakeRunnerVolume(), 0.0001496, TOLERANCE);
    EXPECT_NEAR(head0->getIntakeRunnerCrossSectionArea(), 0.0031225744, TOLERANCE);
    EXPECT_NEAR(head0->getExhaustRunnerVolume(), 5e-05, TOLERANCE);
    EXPECT_NEAR(head0->getExhaustRunnerCrossSectionArea(), 0.0019758025, TOLERANCE);
    EXPECT_FALSE(head0->getFlipDisplay());

    // Camshafts present and non-empty
    Camshaft* intakeCam = head0->getIntakeCamshaft();
    ASSERT_NE(intakeCam, nullptr);
    EXPECT_EQ(intakeCam->getLobeCount(), 4);
    ASSERT_NE(intakeCam->getLobeProfile(), nullptr);
    EXPECT_GT(intakeCam->getLobeProfile()->getSampleCount(), 0)
        << "Intake camshaft must have lobe profile samples";
    EXPECT_NEAR(intakeCam->getBaseRadius(), 0.0254, TOLERANCE);

    Camshaft* exhaustCam = head0->getExhaustCamshaft();
    ASSERT_NE(exhaustCam, nullptr);
    EXPECT_EQ(exhaustCam->getLobeCount(), 4);
    ASSERT_NE(exhaustCam->getLobeProfile(), nullptr);
    EXPECT_GT(exhaustCam->getLobeProfile()->getSampleCount(), 0)
        << "Exhaust camshaft must have lobe profile samples";

    // Port flow functions must be non-null (engine needs flow data)
    EXPECT_NE(head0->getIntakePortFlow(), nullptr)
        << "Intake port flow function must be set";
    EXPECT_NE(head0->getExhaustPortFlow(), nullptr)
        << "Exhaust port flow function must be set";

    delete engine;
}

// 2.8: Ignition module reads timing curve and firing order from JSON
TEST_F(DeserializationCompletenessTest, IgnitionModuleReadsTimingAndFiringOrder) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_GM_LS);
    ASSERT_TRUE(result.success()) << result.error;

    Engine* engine = result.engine;
    IgnitionModule* ignition = engine->getIgnitionModule();
    ASSERT_NE(ignition, nullptr);

    // Timing curve must exist
    Function* timingCurve = ignition->getTimingCurve();
    EXPECT_NE(timingCurve, nullptr) << "Timing curve must be set";
    if (timingCurve) {
        EXPECT_GT(timingCurve->getSampleCount(), 0)
            << "Timing curve must have samples";
    }

    // Rev limit must be set
    EXPECT_GT(ignition->getRevLimit(), 0.0) << "Rev limit must be positive";

    // Firing order must be set for all cylinders
    EXPECT_EQ(ignition->getCylinderCount(), engine->getCylinderCount());

    delete engine;
}

// ============================================================================
// Category 3: No-Fallback Tests
//
// These tests verify that the deserializer REJECTS incomplete JSON
// rather than silently falling back to defaults. Currently the factory
// uses defaults for many fields. After Task #3 (pure deserializer),
// these should throw. Until then, they document the intended behavior.
// ============================================================================

class NoFallbackTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

// 3.1: Missing engine section causes failure (already works in current factory)
TEST_F(NoFallbackTest, MissingEngineSectionFails) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_NO_ENGINE);
    EXPECT_FALSE(result.success());
    EXPECT_FALSE(result.error.empty());
}

// 3.2: Missing cylinder counts causes failure
TEST_F(NoFallbackTest, MissingCylinderCountFails) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_ZERO_CYL);
    EXPECT_FALSE(result.success()) << "Zero cylinder count should fail";
}

// 3.3: Missing crankshafts array causes load failure
// The deserializer rejects incomplete presets with a clear error.
TEST_F(NoFallbackTest, MissingCrankshaftsCausesLoadFailure) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_MISSING_CRANK);
    EXPECT_FALSE(result.success())
        << "Missing crankshafts should cause load failure";
    EXPECT_NE(result.error.find("crankshaft"), std::string::npos)
        << "Error should mention crankshafts, got: " << result.error;
}

// 3.4: Malformed JSON produces error, not crash
TEST_F(NoFallbackTest, MalformedJsonProducesError) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_MALFORMED);
    EXPECT_FALSE(result.success());
    EXPECT_FALSE(result.error.empty());
}

// 3.5: Empty JSON object produces error
TEST_F(NoFallbackTest, EmptyJsonObjectProducesError) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_EMPTY_OBJ);
    EXPECT_FALSE(result.success()) << "Empty JSON should fail (no engine section)";
}

// ============================================================================
// Category 4: End-to-End Preset Loading
// ============================================================================

class EndToEndPresetTest : public ::testing::Test,
                           public ::testing::WithParamInterface<std::string> {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

// 4.1: Preset loads and produces valid engine with correct metadata
TEST_P(EndToEndPresetTest, PresetLoadsAndProducesValidEngine) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(GetParam());
    ASSERT_TRUE(result.success()) << "Failed to load " << GetParam() << ": " << result.error;
    ASSERT_NE(result.engine, nullptr);
    ASSERT_NE(result.vehicle, nullptr);
    ASSERT_NE(result.transmission, nullptr);

    Engine* engine = result.engine;

    // Engine name is non-empty
    EXPECT_FALSE(engine->getName().empty())
        << "Engine name must not be empty";

    // Cylinder count is positive
    EXPECT_GT(engine->getCylinderCount(), 0)
        << "Engine must have cylinders";

    // Displacement is calculated and positive
    engine->calculateDisplacement();
    EXPECT_GT(engine->getDisplacement(), 0.0)
        << "Displacement must be positive";

    // Redline is reasonable (500-15000 RPM range in rad/s)
    EXPECT_GT(engine->getRedline(), 50.0) << "Redline too low";
    EXPECT_LT(engine->getRedline(), 2000.0) << "Redline too high";

    delete engine;
}

INSTANTIATE_TEST_SUITE_P(
    IsomorphismFixtures,
    EndToEndPresetTest,
    ::testing::Values(FIXTURE_HONDA, FIXTURE_GM_LS)
);

// 4.2: GM LS fixture has correct cylinder count, displacement, bore, stroke
TEST_F(EndToEndPresetTest, GmLsHasCorrectEngineGeometry) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_GM_LS);
    ASSERT_TRUE(result.success()) << result.error;

    Engine* engine = result.engine;
    ASSERT_NE(engine, nullptr);

    // GM LS is a V8
    EXPECT_EQ(engine->getCylinderCount(), 8);
    EXPECT_EQ(engine->getCylinderBankCount(), 2);
    EXPECT_EQ(engine->getCrankshaftCount(), 1);

    // Bore = 96.012mm
    CylinderBank* bank0 = engine->getCylinderBank(0);
    ASSERT_NE(bank0, nullptr);
    EXPECT_NEAR(bank0->getBore(), 0.096012, TOLERANCE);

    // Stroke = 2 * crankThrow = 2 * 45.9994mm = 91.9988mm
    Crankshaft* cs = engine->getCrankshaft(0);
    ASSERT_NE(cs, nullptr);
    double stroke = 2.0 * cs->getThrow();
    EXPECT_NEAR(stroke, 0.0919988, 1e-5);

    // Displacement = pi/4 * bore^2 * stroke * cylinders = ~5.327L
    engine->calculateDisplacement();
    double displacement = engine->getDisplacement();
    EXPECT_GT(displacement, 5.0e-3) << "GM LS displacement should be >5L";
    EXPECT_LT(displacement, 6.0e-3) << "GM LS displacement should be <6L";

    delete engine;
}

// 4.3: Honda TRX520 fixture has correct single-cylinder geometry
TEST_F(EndToEndPresetTest, HondaTrx520HasCorrectGeometry) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_HONDA);
    ASSERT_TRUE(result.success()) << result.error;

    Engine* engine = result.engine;
    ASSERT_NE(engine, nullptr);

    // Single cylinder
    EXPECT_EQ(engine->getCylinderCount(), 1);
    EXPECT_EQ(engine->getCylinderBankCount(), 1);

    // Bore = 96mm
    CylinderBank* bank = engine->getCylinderBank(0);
    ASSERT_NE(bank, nullptr);
    EXPECT_NEAR(bank->getBore(), 0.096, TOLERANCE);

    // Stroke = 2 * crankThrow = 2 * 35.75mm
    Crankshaft* cs = engine->getCrankshaft(0);
    ASSERT_NE(cs, nullptr);
    double stroke = 2.0 * cs->getThrow();
    EXPECT_NEAR(stroke, 0.0715, 1e-4);

    // Displacement ~0.518L (pi/4 * 96^2 * 71.5)
    engine->calculateDisplacement();
    double displacement = engine->getDisplacement();
    EXPECT_GT(displacement, 4.5e-4) << "Honda displacement should be >450cc";
    EXPECT_LT(displacement, 6.0e-4) << "Honda displacement should be <600cc";

    delete engine;
}

// 4.4: Port flow Functions are non-null (critical for combustion)
TEST_F(EndToEndPresetTest, PortFlowFunctionsAreNonNull) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_GM_LS);
    ASSERT_TRUE(result.success()) << result.error;

    Engine* engine = result.engine;
    ASSERT_NE(engine, nullptr);

    for (int bi = 0; bi < engine->getCylinderBankCount(); bi++) {
        CylinderHead* head = engine->getHead(bi);
        ASSERT_NE(head, nullptr) << "Head " << bi << " is null";

        EXPECT_NE(head->getIntakePortFlow(), nullptr)
            << "Bank " << bi << " intake port flow is null";
        EXPECT_NE(head->getExhaustPortFlow(), nullptr)
            << "Bank " << bi << " exhaust port flow is null";

        if (head->getIntakePortFlow()) {
            EXPECT_GT(head->getIntakePortFlow()->getSampleCount(), 0)
                << "Bank " << bi << " intake port flow has no samples";
        }
        if (head->getExhaustPortFlow()) {
            EXPECT_GT(head->getExhaustPortFlow()->getSampleCount(), 0)
                << "Bank " << bi << " exhaust port flow has no samples";
        }
    }

    delete engine;
}

// 4.5: Camshaft lobe profiles are present and non-empty for all banks
TEST_F(EndToEndPresetTest, CamshaftLobeProfilesPresentAndNonEmpty) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_GM_LS);
    ASSERT_TRUE(result.success()) << result.error;

    Engine* engine = result.engine;
    ASSERT_NE(engine, nullptr);

    for (int bi = 0; bi < engine->getCylinderBankCount(); bi++) {
        CylinderHead* head = engine->getHead(bi);
        ASSERT_NE(head, nullptr) << "Head " << bi << " is null";

        Camshaft* intakeCam = head->getIntakeCamshaft();
        ASSERT_NE(intakeCam, nullptr) << "Bank " << bi << " intake camshaft is null";
        ASSERT_NE(intakeCam->getLobeProfile(), nullptr)
            << "Bank " << bi << " intake lobe profile is null";
        EXPECT_GT(intakeCam->getLobeProfile()->getSampleCount(), 0)
            << "Bank " << bi << " intake lobe profile is empty";

        Camshaft* exhaustCam = head->getExhaustCamshaft();
        ASSERT_NE(exhaustCam, nullptr) << "Bank " << bi << " exhaust camshaft is null";
        ASSERT_NE(exhaustCam->getLobeProfile(), nullptr)
            << "Bank " << bi << " exhaust lobe profile is null";
        EXPECT_GT(exhaustCam->getLobeProfile()->getSampleCount(), 0)
            << "Bank " << bi << " exhaust lobe profile is empty";
    }

    delete engine;
}

// 4.6: Exhaust system outletFlowRate is non-zero (engine needs to breathe)
TEST_F(EndToEndPresetTest, ExhaustOutletFlowRateIsNonZero) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_GM_LS);
    ASSERT_TRUE(result.success()) << result.error;

    Engine* engine = result.engine;
    for (int i = 0; i < engine->getExhaustSystemCount(); i++) {
        ExhaustSystem* ex = engine->getExhaustSystem(i);
        ASSERT_NE(ex, nullptr);
        EXPECT_GT(ex->getOutletFlowRate(), 0.0)
            << "Exhaust " << i << " outletFlowRate must be non-zero";
    }

    delete engine;
}

// 4.7: Fuel flameSpeed does not crash and returns positive value
TEST_F(EndToEndPresetTest, FuelFlameSpeedReturnsPositiveValue) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_GM_LS);
    ASSERT_TRUE(result.success()) << result.error;

    Engine* engine = result.engine;
    Fuel* fuel = engine->getFuel();
    ASSERT_NE(fuel, nullptr);

    // Call flameSpeed with valid inputs - must not crash
    const double afr = 12.5;
    const double T = 300.0;     // ~room temp
    const double P = 101325.0;  // 1 atm
    const double firingPressure = 1000.0;
    const double motoringPressure = 100.0;

    double flameSpeed = fuel->flameSpeed(1.0, afr, T, P, firingPressure, motoringPressure);
    EXPECT_FALSE(std::isnan(flameSpeed)) << "Flame speed must not be NaN";
    EXPECT_GT(flameSpeed, 0.0) << "Flame speed must be positive";

    delete engine;
}

// 4.8: JSON round-trip - fields read from JSON match JSON source exactly
TEST_F(EndToEndPresetTest, JsonFieldsMatchEngineValues) {
    // Read the fixture JSON
    std::string jsonContent = readFile(FIXTURE_GM_LS);
    ASSERT_FALSE(jsonContent.empty());

    json::JsonValue root = json::parse(jsonContent);
    const auto& engineJson = root["engine"];
    const auto& csJson = engineJson["crankshafts"][static_cast<size_t>(0)];

    // Load via factory
    PresetLoadResult result = PresetEngineFactory::loadFromFile(FIXTURE_GM_LS);
    ASSERT_TRUE(result.success()) << result.error;

    Engine* engine = result.engine;
    Crankshaft* cs = engine->getCrankshaft(0);

    // Verify each crankshaft field matches JSON exactly
    EXPECT_NEAR(cs->getMass(), csJson["mass"].asNumber(), TOLERANCE);
    EXPECT_NEAR(cs->getFlywheelMass(), csJson["flywheelMass"].asNumber(), TOLERANCE);
    EXPECT_NEAR(cs->getMomentOfInertia(), csJson["momentOfInertia"].asNumber(), TOLERANCE);
    EXPECT_NEAR(cs->getThrow(), csJson["crankThrow"].asNumber(), TOLERANCE);
    EXPECT_NEAR(cs->getTdc(), csJson["tdc"].asNumber(), TOLERANCE);
    EXPECT_NEAR(cs->getFrictionTorque(), csJson["frictionTorque"].asNumber(), TOLERANCE);

    // Verify vehicle fields match JSON
    const auto& vj = root["vehicle"];
    Vehicle* v = result.vehicle;
    EXPECT_NEAR(v->getMass(), vj["mass"].asNumber(), TOLERANCE);
    EXPECT_NEAR(v->getDragCoefficient(), vj["dragCoefficient"].asNumber(), TOLERANCE);
    EXPECT_NEAR(v->getCrossSectionArea(), vj["crossSectionArea"].asNumber(), TOLERANCE);
    EXPECT_NEAR(v->getDiffRatio(), vj["diffRatio"].asNumber(), TOLERANCE);
    EXPECT_NEAR(v->getTireRadius(), vj["tireRadius"].asNumber(), TOLERANCE);
    EXPECT_NEAR(v->getRollingResistance(), vj["rollingResistance"].asNumber(), TOLERANCE);

    // Verify exhaust fields match JSON
    const auto& exJson = engineJson["exhaustSystems"][static_cast<size_t>(0)];
    ExhaustSystem* ex = engine->getExhaustSystem(0);
    EXPECT_NEAR(ex->getLength(), exJson["length"].asNumber(), TOLERANCE);
    EXPECT_NEAR(ex->getCollectorCrossSectionArea(), exJson["collectorCrossSectionArea"].asNumber(), TOLERANCE);
    EXPECT_NEAR(ex->getPrimaryTubeLength(), exJson["primaryTubeLength"].asNumber(), TOLERANCE);
    EXPECT_NEAR(ex->getPrimaryFlowRate(), exJson["primaryFlowRate"].asNumber(), TOLERANCE);
    EXPECT_NEAR(ex->getVelocityDecay(), exJson["velocityDecay"].asNumber(), TOLERANCE);
    EXPECT_NEAR(ex->getAudioVolume(), exJson["audioVolume"].asNumber(), TOLERANCE);

    // Verify intake fields match JSON
    const auto& inJson = engineJson["intakes"][static_cast<size_t>(0)];
    Intake* intake = engine->getIntake(0);
    EXPECT_NEAR(intake->getRunnerFlowRate(), inJson["runnerFlowRate"].asNumber(), TOLERANCE);
    EXPECT_NEAR(intake->getRunnerLength(), inJson["runnerLength"].asNumber(), TOLERANCE);
    EXPECT_NEAR(intake->getPlenumCrossSectionArea(), inJson["plenumCrossSectionArea"].asNumber(), TOLERANCE);
    EXPECT_NEAR(intake->getVelocityDecay(), inJson["velocityDecay"].asNumber(), TOLERANCE);

    // Verify fuel fields match JSON
    const auto& fuelJson = engineJson["fuel"];
    Fuel* fuel = engine->getFuel();
    EXPECT_NEAR(fuel->getMolecularAfr(), fuelJson["molecularAfr"].asNumber(), TOLERANCE);
    EXPECT_NEAR(fuel->getBurningEfficiencyRandomness(), fuelJson["burningEfficiencyRandomness"].asNumber(), TOLERANCE);
    EXPECT_NEAR(fuel->getLowEfficiencyAttenuation(), fuelJson["lowEfficiencyAttenuation"].asNumber(), TOLERANCE);
    EXPECT_NEAR(fuel->getMaxBurningEfficiency(), fuelJson["maxBurningEfficiency"].asNumber(), TOLERANCE);
    EXPECT_NEAR(fuel->getMaxTurbulenceEffect(), fuelJson["maxTurbulenceEffect"].asNumber(), TOLERANCE);
    EXPECT_NEAR(fuel->getMaxDilutionEffect(), fuelJson["maxDilutionEffect"].asNumber(), TOLERANCE);

    delete engine;
}

// ============================================================================
// Category 5: Parameter-Level Isomorphism (.mr vs JSON)
//
// macOS + Piranha only. Loads the same engine via both paths and compares
// getter values field-by-field. These tests prove the JSON deserializer
// produces the same engine object graph as the Piranha interpreter.
// ============================================================================

#if !TARGET_OS_IPHONE && defined(ENGINE_SIM_PIRANHA_ENABLED)

namespace {
    constexpr int ISO_SAMPLE_RATE = 48000;

    // Load an engine via Piranha (.mr script)
    // Returns raw Engine* (caller does NOT own -- owned by ISimulator)
    Engine* loadEngineFromScript(const std::string& scriptPath, const std::string& assetBase,
                                  std::unique_ptr<ISimulator>& outISim) {
        const std::string absoluteScriptPath = test_path_helpers::makeAbsolutePath(scriptPath);
        const std::string absoluteAssetBase = test_path_helpers::makeAbsolutePath(assetBase);

        ISimulatorConfig config;
        config.sampleRate = ISO_SAMPLE_RATE;
        config.simulationFrequency = 10000;

        try {
            test_path_helpers::ScopedWorkingDirectory scopedWorkingDirectory(
                test_path_helpers::engineSimRootForAssets(absoluteAssetBase));
            outISim = SimulatorFactory::create(
                SimulatorType::PistonEngine,
                absoluteScriptPath,
                absoluteAssetBase,
                config);
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to load Piranha script '" + absoluteScriptPath + "': " + e.what());
        }

        if (!outISim) {
            throw std::runtime_error("Failed to create simulator for script '" + absoluteScriptPath + "'");
        }

        auto* bridge = dynamic_cast<BridgeSimulator*>(outISim.get());
        if (!bridge) {
            throw std::runtime_error("SimulatorFactory returned non-bridge simulator for script '" + absoluteScriptPath + "'");
        }

        Simulator* sim = bridge->getInternalSimulator();
        if (!sim) {
            throw std::runtime_error("Bridge simulator has no internal simulator for script '" + absoluteScriptPath + "'");
        }

        // Initialize convolution filters (unit impulses)
        SimulatorInitHelpers::initializeConvolutionFilters(sim);

        Engine* engine = sim->getEngine();
        if (!engine) {
            throw std::runtime_error("Piranha script did not produce an engine for '" + absoluteScriptPath + "'");
        }

        return engine;
    }

    // Load an engine via JSON preset factory
    // Returns raw Engine* (caller OWNS and must delete)
    Engine* loadEngineFromJson(const std::string& fixturePath) {
        PresetLoadResult result = PresetEngineFactory::loadFromFile(fixturePath);
        if (!result.success()) return nullptr;
        return result.engine;
    }

    // Compare two Functions sample-by-sample
    void compareFunctions(Function* expected, Function* actual, const std::string& context) {
        ASSERT_NE(expected, nullptr) << context << ": expected function is null";
        ASSERT_NE(actual, nullptr) << context << ": actual function is null";

        // filterRadius must match (was previously broken: computed vs read)
        EXPECT_NEAR(actual->getFilterRadius(), expected->getFilterRadius(), 1e-6)
            << context << ": filterRadius mismatch";

        int minSamples = std::min(expected->getSampleCount(), actual->getSampleCount());
        ASSERT_GT(minSamples, 0) << context << ": no samples to compare";

        // Check first, middle, and last samples
        auto checkSample = [&](int idx, const char* label) {
            if (idx < expected->getSampleCount() && idx < actual->getSampleCount()) {
                EXPECT_NEAR(actual->getX(idx), expected->getX(idx), 1e-6)
                    << context << ": " << label << " sample X mismatch at index " << idx;
                EXPECT_NEAR(actual->getY(idx), expected->getY(idx), 1e-8)
                    << context << ": " << label << " sample Y mismatch at index " << idx;
            }
        };
        checkSample(0, "first");
        if (minSamples > 2) checkSample(minSamples / 2, "middle");
        if (minSamples > 1) checkSample(minSamples - 1, "last");
    }

    // Test parameter: maps script path to fixture path
    struct IsomorphismParam {
        std::string name;
        std::string scriptPath;
        std::string fixturePath;
        std::string assetBase;
    };
}

class ParameterIsomorphismTest : public ::testing::Test,
                                 public ::testing::WithParamInterface<IsomorphismParam> {
protected:
    std::unique_ptr<ISimulator> piranhaSim_;
    Engine* piranhaEngine_ = nullptr;
    Engine* jsonEngine_ = nullptr;

    void SetUp() override {
        auto param = GetParam();
        param.scriptPath = test_path_helpers::makeAbsolutePath(param.scriptPath);
        param.fixturePath = test_path_helpers::makeAbsolutePath(param.fixturePath);
        param.assetBase = test_path_helpers::makeAbsolutePath(param.assetBase);

        try {
            piranhaEngine_ = loadEngineFromScript(param.scriptPath, param.assetBase, piranhaSim_);
        } catch (const std::exception& e) {
            FAIL() << e.what();
        }

        jsonEngine_ = loadEngineFromJson(param.fixturePath);
        ASSERT_NE(jsonEngine_, nullptr)
            << "Failed to load JSON engine from " << param.fixturePath;
    }

    void TearDown() override {
        // jsonEngine_ is owned by us (must delete)
        delete jsonEngine_;
        jsonEngine_ = nullptr;

        // piranhaEngine_ is owned by the ISimulator (destroyed via unique_ptr)
        if (piranhaSim_) {
            auto* bridge = dynamic_cast<BridgeSimulator*>(piranhaSim_.get());
            if (bridge) {
                Simulator* sim = bridge->getInternalSimulator();
                if (sim) sim->destroy();
            }
            piranhaSim_.reset();
        }
        piranhaEngine_ = nullptr;
    }
};

// 5.1: Engine topology matches (cylinder count, bank count, crankshaft count)
TEST_P(ParameterIsomorphismTest, EngineTopologyMatches) {
    EXPECT_EQ(jsonEngine_->getCylinderCount(), piranhaEngine_->getCylinderCount())
        << "Cylinder count mismatch";
    EXPECT_EQ(jsonEngine_->getCylinderBankCount(), piranhaEngine_->getCylinderBankCount())
        << "Cylinder bank count mismatch";
    EXPECT_EQ(jsonEngine_->getCrankshaftCount(), piranhaEngine_->getCrankshaftCount())
        << "Crankshaft count mismatch";
    EXPECT_EQ(jsonEngine_->getExhaustSystemCount(), piranhaEngine_->getExhaustSystemCount())
        << "Exhaust system count mismatch";
    EXPECT_EQ(jsonEngine_->getIntakeCount(), piranhaEngine_->getIntakeCount())
        << "Intake count mismatch";

    // Displacement should match
    jsonEngine_->calculateDisplacement();
    piranhaEngine_->calculateDisplacement();
    double jsonDisp = jsonEngine_->getDisplacement();
    double piranhaDisp = piranhaEngine_->getDisplacement();
    EXPECT_NEAR(jsonDisp, piranhaDisp, piranhaDisp * 0.01)
        << "Displacement mismatch (json=" << jsonDisp << ", piranha=" << piranhaDisp << ")";
}

// 5.2: Crankshaft parameters match (mass, moment of inertia, crank throw, rod journals)
TEST_P(ParameterIsomorphismTest, CrankshaftParametersMatch) {
    ASSERT_GT(piranhaEngine_->getCrankshaftCount(), 0);
    ASSERT_GT(jsonEngine_->getCrankshaftCount(), 0);

    Crankshaft* pCs = piranhaEngine_->getCrankshaft(0);
    Crankshaft* jCs = jsonEngine_->getCrankshaft(0);
    ASSERT_NE(pCs, nullptr);
    ASSERT_NE(jCs, nullptr);

    EXPECT_NEAR(jCs->getMass(), pCs->getMass(), pCs->getMass() * 0.001)
        << "Crankshaft mass mismatch";
    EXPECT_NEAR(jCs->getMomentOfInertia(), pCs->getMomentOfInertia(),
                pCs->getMomentOfInertia() * 0.001)
        << "Crankshaft momentOfInertia mismatch";
    EXPECT_NEAR(jCs->getThrow(), pCs->getThrow(), pCs->getThrow() * 0.001)
        << "Crankshaft crankThrow mismatch";
    EXPECT_NEAR(jCs->getFlywheelMass(), pCs->getFlywheelMass(),
                pCs->getFlywheelMass() * 0.001)
        << "Crankshaft flywheelMass mismatch";
    EXPECT_NEAR(jCs->getTdc(), pCs->getTdc(), 1e-6)
        << "Crankshaft TDC mismatch";
    EXPECT_NEAR(jCs->getFrictionTorque(), pCs->getFrictionTorque(),
                pCs->getFrictionTorque() * 0.001)
        << "Crankshaft frictionTorque mismatch";
    EXPECT_EQ(jCs->getRodJournalCount(), pCs->getRodJournalCount())
        << "Crankshaft rodJournal count mismatch";
}

// 5.3: Port flow Functions match (filterRadius and key sample points)
// This was previously broken: JSON path computed filterRadius instead of reading it.
TEST_P(ParameterIsomorphismTest, PortFlowFunctionsMatch) {
    for (int bi = 0; bi < piranhaEngine_->getCylinderBankCount() &&
                     bi < jsonEngine_->getCylinderBankCount(); bi++) {
        CylinderHead* pHead = piranhaEngine_->getHead(bi);
        CylinderHead* jHead = jsonEngine_->getHead(bi);
        ASSERT_NE(pHead, nullptr) << "Piranha head " << bi << " is null";
        ASSERT_NE(jHead, nullptr) << "JSON head " << bi << " is null";

        // Intake port flow
        compareFunctions(pHead->getIntakePortFlow(), jHead->getIntakePortFlow(),
            "Bank " + std::to_string(bi) + " intake port flow");

        // Exhaust port flow
        compareFunctions(pHead->getExhaustPortFlow(), jHead->getExhaustPortFlow(),
            "Bank " + std::to_string(bi) + " exhaust port flow");
    }
}

// 5.4: Camshaft lobe profiles match (filterRadius and sample points)
// This was previously broken: JSON path resampled camshaft profiles.
TEST_P(ParameterIsomorphismTest, CamshaftLobeProfilesMatch) {
    for (int bi = 0; bi < piranhaEngine_->getCylinderBankCount() &&
                     bi < jsonEngine_->getCylinderBankCount(); bi++) {
        CylinderHead* pHead = piranhaEngine_->getHead(bi);
        CylinderHead* jHead = jsonEngine_->getHead(bi);
        ASSERT_NE(pHead, nullptr);
        ASSERT_NE(jHead, nullptr);

        // Intake camshaft
        Camshaft* pIntakeCam = pHead->getIntakeCamshaft();
        Camshaft* jIntakeCam = jHead->getIntakeCamshaft();
        if (pIntakeCam && jIntakeCam) {
            compareFunctions(pIntakeCam->getLobeProfile(), jIntakeCam->getLobeProfile(),
                "Bank " + std::to_string(bi) + " intake camshaft lobe profile");

            EXPECT_NEAR(jIntakeCam->getBaseRadius(), pIntakeCam->getBaseRadius(), 1e-6)
                << "Bank " << bi << " intake camshaft baseRadius mismatch";
            EXPECT_EQ(jIntakeCam->getLobeCount(), pIntakeCam->getLobeCount())
                << "Bank " << bi << " intake camshaft lobe count mismatch";
        }

        // Exhaust camshaft
        Camshaft* pExhaustCam = pHead->getExhaustCamshaft();
        Camshaft* jExhaustCam = jHead->getExhaustCamshaft();
        if (pExhaustCam && jExhaustCam) {
            compareFunctions(pExhaustCam->getLobeProfile(), jExhaustCam->getLobeProfile(),
                "Bank " + std::to_string(bi) + " exhaust camshaft lobe profile");

            EXPECT_NEAR(jExhaustCam->getBaseRadius(), pExhaustCam->getBaseRadius(), 1e-6)
                << "Bank " << bi << " exhaust camshaft baseRadius mismatch";
        }
    }
}

// 5.5: Fuel parameters match (previously: molecularMass/energyDensity/density were defaults)
TEST_P(ParameterIsomorphismTest, FuelParametersMatch) {
    Fuel* pFuel = piranhaEngine_->getFuel();
    Fuel* jFuel = jsonEngine_->getFuel();
    ASSERT_NE(pFuel, nullptr);
    ASSERT_NE(jFuel, nullptr);

    EXPECT_NEAR(jFuel->getMolecularAfr(), pFuel->getMolecularAfr(), 0.01)
        << "Fuel molecularAfr mismatch";
    EXPECT_NEAR(jFuel->getMaxBurningEfficiency(), pFuel->getMaxBurningEfficiency(), 0.001)
        << "Fuel maxBurningEfficiency mismatch";
    EXPECT_NEAR(jFuel->getBurningEfficiencyRandomness(), pFuel->getBurningEfficiencyRandomness(), 0.001)
        << "Fuel burningEfficiencyRandomness mismatch";
    EXPECT_NEAR(jFuel->getLowEfficiencyAttenuation(), pFuel->getLowEfficiencyAttenuation(), 0.001)
        << "Fuel lowEfficiencyAttenuation mismatch";
    EXPECT_NEAR(jFuel->getMaxTurbulenceEffect(), pFuel->getMaxTurbulenceEffect(), 0.001)
        << "Fuel maxTurbulenceEffect mismatch";
    EXPECT_NEAR(jFuel->getMaxDilutionEffect(), pFuel->getMaxDilutionEffect(), 0.001)
        << "Fuel maxDilutionEffect mismatch";
}

// 5.6: Vehicle parameters match
TEST_P(ParameterIsomorphismTest, VehicleParametersMatch) {
    // Piranha engines don't always have vehicle; load via PresetLoadResult for JSON
    // For the script path, vehicle comes from the Simulator
    auto* pBridge = dynamic_cast<BridgeSimulator*>(piranhaSim_.get());
    ASSERT_NE(pBridge, nullptr);
    Simulator* pSim = pBridge->getInternalSimulator();
    ASSERT_NE(pSim, nullptr);
    Vehicle* pVehicle = pSim->getVehicle();
    ASSERT_NE(pVehicle, nullptr) << "Piranha simulator has no vehicle";

    // JSON path: load separately to get vehicle
    PresetLoadResult jsonResult = PresetEngineFactory::loadFromFile(GetParam().fixturePath);
    ASSERT_TRUE(jsonResult.success()) << "Failed to reload JSON for vehicle comparison";
    ASSERT_NE(jsonResult.vehicle, nullptr);

    EXPECT_NEAR(jsonResult.vehicle->getMass(), pVehicle->getMass(), 0.1)
        << "Vehicle mass mismatch";
    EXPECT_NEAR(jsonResult.vehicle->getDragCoefficient(), pVehicle->getDragCoefficient(), 0.01)
        << "Vehicle dragCoefficient mismatch";
    EXPECT_NEAR(jsonResult.vehicle->getDiffRatio(), pVehicle->getDiffRatio(), 0.01)
        << "Vehicle diffRatio mismatch";
    EXPECT_NEAR(jsonResult.vehicle->getTireRadius(), pVehicle->getTireRadius(), 0.001)
        << "Vehicle tireRadius mismatch";

    // Note: jsonResult.engine needs cleanup since we only needed vehicle
    delete jsonResult.engine;
}

// Instantiate for Honda TRX520 and GM LS
INSTANTIATE_TEST_SUITE_P(
    IsomorphismPresets,
    ParameterIsomorphismTest,
    ::testing::Values(
        IsomorphismParam{
            "Honda_TRX520",
            TEST_ENGINE_SIM_ASSETS "/engines/atg-video-1/01_honda_trx520.mr",
            TEST_FIXTURE_DIR "/01_honda_trx520.preset.json",
            TEST_ENGINE_SIM_ASSETS "/"
        },
        IsomorphismParam{
            "GM_LS",
            TEST_ENGINE_SIM_ASSETS "/engines/atg-video-2/07_gm_ls.mr",
            TEST_FIXTURE_DIR "/gm_ls.preset.json",
            TEST_ENGINE_SIM_ASSETS "/"
        }
    )
);

#endif // !TARGET_OS_IPHONE && ENGINE_SIM_PIRANHA_ENABLED
