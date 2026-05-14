#ifndef ATG_ENGINE_SIM_ENGINE_SIM_APPLICATION_H
#define ATG_ENGINE_SIM_ENGINE_SIM_APPLICATION_H

#include "geometry_generator.h"
#include "simulator.h"
#include "engine.h"
#include "simulation_object.h"
#include "ui_manager.h"
#include "dynamometer.h"
#include "oscilloscope.h"
#include "audio_buffer.h"
#include "convolution_filter.h"
#include "shaders.h"
#include "engine_view.h"
#include "right_gauge_cluster.h"
#include "cylinder_temperature_gauge.h"
#include "synthesizer.h"
#include "oscilloscope_cluster.h"
#include "performance_cluster.h"
#include "load_simulation_cluster.h"
#include "mixer_cluster.h"
#include "info_cluster.h"
#include "labeled_gauge.h"
#include "application_settings.h"
#include "transmission.h"
#include "delta.h"
#include "dtv.h"
#include "video_capture.h"

#include <vector>
#include <string>

class EngineSimApplication {
    private:
        static std::string s_buildVersion = "0.1.12a";

#ifdef ATG_ENGINE_SIM_PRESET_MODE
        bool m_presetMode = true;
#else
        bool m_presetMode = false;
#endif

    public:
        EngineSimApplication();
        virtual ~EngineSimApplication();

        static std::string getBuildVersion() { return s_buildVersion; }

        void initialize(void *instance, ysContextObject::DeviceAPI api);
        void run();
        void destroy();

        void loadEngine(Engine *engine, Vehicle *vehicle, Transmission *transmission);
        void drawGenerated(
                const GeometryGenerator::GeometryIndices &indices,
                int layer = 0);
        void drawGeneratedUi(
                const GeometryGenerator::GeometryIndices &indices,
                int layer);
        void configure(const ApplicationSettings &settings);

        GeometryGenerator *getGeometryGenerator() { return &m_geometryGenerator; }

    protected:
        double m_speedSetting = 1.0;
        double m_targetSpeedSetting = 1.0;

        // ... (rest of existing members unchanged)
};
