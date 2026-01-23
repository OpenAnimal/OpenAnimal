
<p align="center">
  <img src="https://raw.githubusercontent.com/OpenAnimal/OpenAnimal/main/Assets/Media/OA_raptor6_flat.png" alt="OA raptor flat" width="600" />
</p>

<h1 align="center">OpenAnimal – OpenBrain - Emotional Robots and Droids 🦖</h1>


<p align="center">
  <a href="...">🧠 Neural Networks</a> |
  <a href="...">📚 LLM/VLM</a> |
  <a href="...">🤖 Robot Hardware</a> |
  <a href="...">📱 Android</a>
</p>



## Overview

OpenAnimal is an extensible Unreal Engine–based framework for modeling and simulating animal- and human-like behavior. It integrates neural networks, behavior modeling, speech processing, and optional robotics interfaces directly within Unreal Engine.

OpenBrain is a self-contained, plug-and-play Animal Brain system designed to bring autonomous behavior to any Actor in Unreal Engine and to physical platforms. This module focuses on extensibility, aiming to simulate natural animal cognition and social interaction (rather than high-level intelligence).

The system is built to run fully offline with the brain on a phone and for affordable hardware.

It is similar to robot operating systems like [OpenBot](https://github.com/ob-f/OpenBot) or [OM1](https://github.com/OpenMind/OM1) but has distinct features and a different philosophy.

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
* Autonomous pets and droids
* Educational scenarios and demonstrations
* Robotics and hardware prototyping
* Research simulations, Social behavior studies and modeling


## Dependencies

* **ONNX Neural Networks**  
  *See:* [Neural Networks README](NeuralNetworks/README.md) for all single-file ONNX networks
  For convenience a **Utility Widget** is provided to automatically download the networks to the correct location. 


### Optional

* [VaRest](https://github.com/AboveConstraints/VaRestX)
Reliable HTTP/URL communication layer for interacting with hardware and large/visual language models.  
(VaRest is the battle-tested solution currently used, but it might be replaced by UE HTTP and JSON.)

* [Ollama](https://ollama.com/)  
For local execution of large and visual language models, including support for cloud-based model computation via the new Ollama cloud-model options.



## Platforms

* Windows 64-bit
* Android 



## Release State and Disclaimer

This is an early-stage release some features are incomplete. My resources are limited, but the project is expected to expand over time. 


## Contact & Support

Email: OpenAnimal@outlook not .com but .de  
For questions that may benefit others, please open an [Issue](https://github.com/OpenAnimal/OpenAnimal/issues) here on GitHub








