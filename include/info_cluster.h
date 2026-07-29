#ifndef ATG_ENGINE_SIM_INFO_CLUSTER_CPP
#define ATG_ENGINE_SIM_INFO_CLUSTER_CPP

#include "ui_element.h"
#include "engine.h"

#include <string>

class InfoCluster : public UiElement {
    public:
        InfoCluster();
        virtual ~InfoCluster();

        virtual void initialize(EngineSimApplication *app);
        virtual void destroy();
        virtual void update(float dt);
        virtual void render();

        void setEngine(Engine *engine) { m_engine = engine; }
        void setLogMessage(const std::string &logMessage) { m_logMessage = logMessage; }
        std::string getLogMessage() const { return m_logMessage; }

#ifdef ATG_ENGINE_SIM_PRESET_MODE
        bool m_isPresetMode = true;
#else
        bool m_isPresetMode = false;
#endif
};

#endif /* ATG_ENGINE_SIM_INFO_CLUSTER_CPP */