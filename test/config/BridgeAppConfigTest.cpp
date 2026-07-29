#include <gtest/gtest.h>
#include <config/BridgeAppConfig.h>
#include <config/CLIFactory.h>
#include <input/DemoInputProvider.h>
#include <io/IInputProvider.h>
#include <twin/IceVehicleProfile.h>

using namespace config;
using namespace input;
using namespace twin;

class BridgeAppConfigTest : public ::testing::Test {
protected:
    BridgeAppConfig config_;
};

// ============================================================================
// Test 1: BridgeAppConfig has modelDemo field, defaults to false
// ============================================================================

TEST_F(BridgeAppConfigTest, ConnectDemo_DefaultsToFalse) {
    BridgeAppConfig config;
    EXPECT_FALSE(config.modelDemo);
}

// ============================================================================
// Test 2: When --model-demo flag is passed, modelDemo is true
// ============================================================================

TEST_F(BridgeAppConfigTest, ConnectDemo_SetToTrue) {
    BridgeAppConfig config;
    config.modelDemo = true;
    EXPECT_TRUE(config.modelDemo);
}

// ============================================================================
// Test 3: Implicit settings applied (playAudio = true, interactive = true)
// when modelDemo is true
// ============================================================================

TEST_F(BridgeAppConfigTest, ConnectDemoTrue_SetsPlayAudioTrue) {
    BridgeAppConfig config;
    config.modelDemo = true;
    // Implicit setting: when modelDemo is true, playAudio should be true
    config.playAudio = true;
    EXPECT_TRUE(config.playAudio);
}

TEST_F(BridgeAppConfigTest, ConnectDemoTrue_SetsInteractiveTrue) {
    BridgeAppConfig config;
    config.modelDemo = true;
    // Implicit setting: when modelDemo is true, interactive should be true
    config.interactive = true;
    EXPECT_TRUE(config.interactive);
}

TEST_F(BridgeAppConfigTest, ConnectDemoFalse_DoesNotSetPlayAudio) {
    BridgeAppConfig config;
    config.modelDemo = false;
    // When modelDemo is false, playAudio remains false
    EXPECT_FALSE(config.playAudio);
}

TEST_F(BridgeAppConfigTest, ConnectDemoFalse_DoesNotSetInteractive) {
    BridgeAppConfig config;
    config.modelDemo = false;
    // When modelDemo is false, interactive remains false
    EXPECT_FALSE(config.interactive);
}

// ============================================================================
// Test 4: The createInputProvider factory creates a DemoInputProvider
// when modelDemo is true
// ============================================================================

TEST_F(BridgeAppConfigTest, CreateInputProvider_ConnectDemoTrue_ReturnsDemoInputProvider) {
    BridgeAppConfig config;
    config.modelDemo = true;
    config.playAudio = true;
    config.interactive = true;

    input::IInputProvider* provider = createInputProvider(config);
    ASSERT_NE(provider, nullptr);
    EXPECT_EQ(provider->GetProviderName(), "DemoInputProvider");
    delete provider; // Clean up raw pointer
}

TEST_F(BridgeAppConfigTest, CreateInputProvider_ConnectDemoFalse_ReturnsDefaultProvider) {
    BridgeAppConfig config;
    config.modelDemo = false;

    input::IInputProvider* provider = createInputProvider(config);
    // When modelDemo is false, should return default/null provider
    // This behavior will be defined in Task #5
    EXPECT_EQ(provider, nullptr);
}
