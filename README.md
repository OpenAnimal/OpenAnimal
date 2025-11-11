
## OpenAnimal – OpenBrain

![OA raptor](https://raw.githubusercontent.com/OpenAnimal/OpenAnimal/main/Assets/Media/OA_raptor6_flat.jpg)


### - **Official Documentation** -

### Overview

OpenAnimal – OpenBrain is an extensible Unreal Engine–based framework for modeling and simulating animal and human-like behavior.
It integrates neural networks, behavior modeling, speech and audio processing, and optional robotics interfaces directly within 
Unreal Engine with a strong emphasis on standalone, autonomous operation. 



### Key Features

* **Native Unreal Engine ONNX neural-network inference in Blueprints**
* **Asynchronous processing pipelines**
* **Speech-to-Text and speaker recognition**
* **Animal pose, face, and gaze detection**  
  Estimate posture, orientation, and visual focus. 
* **Behavior and emotional-state modeling (preliminary)**  
  Early-stage behavior generation and emotion approximation. 
* **Platform-independent ONNX networks**  
  Single-file ONNX models that are openly available and operate consistently across platforms.
* **Bottom-up autonomous intelligence modeling**  
  Designed to build emergent behavior from atomic perceptual and processing components. 
* **Android and Arduino based**  
  Enabling integration with broad variety of hardware setups


### Use Cases
* Pet
* Educational scenarios and demonstrations
* Robotics and hardware prototyping
* Research simulations, Social behavior studies and modeling


### Dependencies Required

* **ONNX Neural Networks**  
  *See:* [Neural Networks README](NeuralNetworks/README.md) for all single-file ONNX networks
  For convenience a **Utility Widget** is provided to automatically download the networks to the correct location. 


### Optional

* [VaRest](`https://github.com/AboveConstraints/VaRestX`) *(optional, but highly recommended)* 
Reliable HTTP/URL communication layer for interacting with hardware and large/visual language models.  
(Unreal Engine’s built-in HTML communication methods are yet too unstable; VaRest is the battle-tested solution currently used.)

* [Ollama](`https://ollama.com/`)  
For local execution of large and visual language models, including support for cloud-based model computation via the new Ollama cloud-model options.




## Platforms

* Windows 64-bit
* Android 



## Release State and Disclaimer

This is an early-stage release and was published under high time pressure. Some features are incomplete, and bugs may exist. Development will continue, and the project is expected to expand over time. 


## Contact & Support

* Email: OpenAnimal@outlook not .com but .de
* For questions that may benefit others, please open an [Issue](https://github.com/OpenAnimal/OpenAnimal/issues) here on GitHub








