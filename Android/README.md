
## Brain on a Android Smartphone


Android Phones are ought to be used to deploy the standalone Brain. 

Compiles for Android:
OpenAnimal  (x)
VaRest      (x) (VaRest might be replaced by upcoming integrated UE tools)

SocketIO    ()  (For USB-Serial Connection. We might port it, but WiFi is the first to go)




So far the OpenAnimal Plugin does compile for Android, as well as VaRest for http, but this doesn't mean that all functionality is provided and tested, as packaging might pass without errors, but essential functionality is unavailable. This will require more testing to verify, eventhough the UE-based features should be smartphone compatible without major drawbacks. 
The SerialCom plugin however is not compatible out-of-the-box, as it uses Windows functions that are silently excluded when packaging. But we plan to make the plugin compatible in the near future. 
