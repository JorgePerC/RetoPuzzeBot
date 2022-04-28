#include "fl/Headers.h"

using namespace fl;

int main(int argc, char const *argv[])
{
    /*
    In this file we import the system from a .ffl file, which is just 
    a fancy txt file to define our control system
    */
    Engine* engine = FllImporter().fromFile("tip.fll");

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

    InputVariable* service = engine->getInputVariable("service");
    OutputVariable* tip = engine->getOutputVariable("tip");

    for (int i = 0; i <= 50; ++i){
        scalar location = service->getMinimum() + i * (service->range() / 50);
        service->setValue(location);
        engine->process();
        FL_LOG("service.input = " << Op::str(location) << 
            " => " << "tip.output = " << Op::str(tip->getValue()));
    }
}
