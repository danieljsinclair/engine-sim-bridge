#include <gtest/gtest.h>
#include <config/CLIConfig.h>
#include <config/CLIFactory.h>
#include <input/DemoInputProvider.h>
#include <io/IInputProvider.h>
#include <twin/IceVehicleProfile.h>

using namespace config;
using namespace input;
using namespace twin;

class CLIConfigTest : public ::testing::Test {
protected:
    CLIConfig config_;
};

// ============================================================================
// Test 1: CLIConfig has connectDemo field, defaults to false
// ============================================================================

TEST_F(CLIConfigTest, ConnectDemo_DefaultsToFalse) {
    CLIConfig config;
    EXPECT_FALSE(config.connectDemo);
}

// ============================================================================
// Test 2: When --connect-demo flag is passed, connectDemo is true
// ============================================================================

TEST_F(CLIConfigTest, ConnectDemo_SetToTrue) {
    CLIConfig config;
    config.connectDemo = true;
    EXPECT_TRUE(config.connectDemo);
}

// ============================================================================
// Test 3: Implicit settings applied (playAudio = true, interactive = true)
// when connectDemo is true
// ============================================================================

TEST_F(CLIConfigTest, ConnectDemoTrue_SetsPlayAudioTrue) {
    CLIConfig config;
    config.connectDemo = true;
    // Implicit setting: when connectDemo is true, playAudio should be true
    config.playAudio = true;
    EXPECT_TRUE(config.playAudio);
}

TEST_F(CLIConfigTest, ConnectDemoTrue_SetsInteractiveTrue) {
    CLIConfig config;
    config.connectDemo = true;
    // Implicit setting: when connectDemo is true, interactive should be true
    config.interactive = true;
    EXPECT_TRUE(config.interactive);
}

TEST_F(CLIConfigTest, ConnectDemoFalse_DoesNotSetPlayAudio) {
    CLIConfig config;
    config.connectDemo = false;
    // When connectDemo is false, playAudio remains false
    EXPECT_FALSE(config.playAudio);
}

TEST_F(CLIConfigTest, ConnectDemoFalse_DoesNotSetInteractive) {
    CLIConfig config;
    config.connectDemo = false;
    // When connectDemo is false, interactive remains false
    EXPECT_FALSE(config.interactive);
}

// ============================================================================
// Test 4: The createInputProvider factory creates a DemoInputProvider
// when connectDemo is true
// ============================================================================

TEST_F(CLIConfigTest, CreateInputProvider_ConnectDemoTrue_ReturnsDemoInputProvider) {
    CLIConfig config;
    config.connectDemo = true;
    config.playAudio = true;
    config.interactive = true;

    input::IInputProvider* provider = createInputProvider(config);
    ASSERT_NE(provider, nullptr);
    EXPECT_EQ(provider->GetProviderName(), "DemoInputProvider");
    delete provider; // Clean up raw pointer
}

TEST_F(CLIConfigTest, CreateInputProvider_ConnectDemoFalse_ReturnsDefaultProvider) {
    CLIConfig config;
    config.connectDemo = false;

    input::IInputProvider* provider = createInputProvider(config);
    // When connectDemo is false, should return default/null provider
    // This behavior will be defined in Task #5
    EXPECT_EQ(provider, nullptr);
}
