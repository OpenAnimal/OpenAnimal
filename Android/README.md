
## Brain on an Android Smartphone

Android phones are intended as a primary deployment target for running the standalone “brain” component. However for development and testing, f.i. with windows, one can directly connect the OpenBrain_BP to the ESP32 via Wifi. The goal is to run perception, behavior generation, and communication autonomously. 


### Current Android Compilation Status

* ✅ **OpenAnimal:** Compiles for Android
* ✅ **VaRest:** Compiles for Android and provides reliable HTTP communication for LLM/VLMs


### Notes on Mobile Functionality

OpenAnimal and VaRest both compile for Android successfully; however, this does not guarantee complete feature availability at runtime. In some cases, packaging may succeed but specific functionality (e.g., network-level or file-level capabilities) 'may' silently not behave identically to desktop, but this is expected to be minor cases, as features and modules that rely solely on Unreal Engine systems should generally be compatible with smartphones devices without major limitations. 

#### USB-Serial (depricated) Status

* ❌ **SerialCOM:** Minor as USB-Connection is currently not used. Wi-Fi-based communication is preferred

The SerialCom plugin does not support Android at this stage due to its reliance on Windows functions that are silently excluded when packaging. The intention is to make it potentially compatible in the future, but it is currently not part of the Android workflow, but kept for completeness. 


---
