#include <chrono/core/ChDataPath.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono_vsg/ChVisualSystemVSG.h>

#include <cstdlib>
#include <iostream>

int main() {
#ifdef CHRONO_DATA_DIR
    chrono::SetChronoDataPath(CHRONO_DATA_DIR);
#endif
    std::cout << "data=" << chrono::GetChronoDataPath() << std::endl;
    std::cout << "VK_DRIVER_FILES=" << (std::getenv("VK_DRIVER_FILES") ? std::getenv("VK_DRIVER_FILES") : "") << std::endl;
    std::cout << "VK_ICD_FILENAMES=" << (std::getenv("VK_ICD_FILENAMES") ? std::getenv("VK_ICD_FILENAMES") : "") << std::endl;
    std::cout.flush();

    chrono::ChSystemNSC sys;
    auto vis = chrono_types::make_shared<chrono::vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("VSG smoke (Arc A370M)");
    vis->SetWindowPosition(80, 80);
    vis->SetCameraVertical(chrono::CameraVerticalDir::Y);

    std::cout << "calling Initialize" << std::endl;
    std::cout.flush();
    vis->Initialize();
    std::cout << "Initialize returned" << std::endl;
    std::cout.flush();

    vis->AddCamera(chrono::ChVector3d(-2, 1.5, -2), chrono::ChVector3d(0, 0.2, 0));
    vis->BindAll();

    int frames = 0;
    while (vis->Run() && frames < 180) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(0.01);
        ++frames;
    }
    std::cout << "ok frames=" << frames << std::endl;
    return 0;
}
