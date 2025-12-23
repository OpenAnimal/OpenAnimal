
## Large Language Models and Visual Language Models

The plugin supports multiple levels of cognition, ranging from low-level socio-dynamic behavior to higher-level reasoning and analysis. For advanced cognitive layers, integration with Large Language Models (LLMs) and Visual Language Models (VLMs) is provided. 

### Local LLM/VLM Execution via Ollama

To maintain the plugin’s philosophy of supporting standalone, offline, and self-contained operation, Ollama is used as the local provider for LLM and VLM inference. Ollama allows hosting both lightweight and complex models locally with minimal setup effort. 

Download [Ollama](https://ollama.com/)

### Blueprint-Based Prompt Generation

The plugin includes Blueprint utilities for constructing prompts and specifying structured return types (e.g., `JSON`). Returned data can automatically populate Blueprint variables, including arrays and typed structures. 

Images / RenderTargets can be encoded to `Base64` in Blueprint via dedicated utility functions, enabling image inputs for visual models. 

The LLM/VLM pipeline is implemented as an Actor Component that can be attached to the “brain,” with extensive inline instructions and example prompts included. 

### Communication Layer

Communication with Ollama is performed through the `VaRest` plugin. VaRest provides a robust `HTTP REST` interface that proved more stable than Unreal’s internal `JSON/HTTP` nodes, which were unreliable during testing, but it is planned to convert to the in-house methods at some point. 

### Known Limitations
* Porting Ollama onto a smartphone might yet not be as straightforward as for desktop.
* Model behavior varies depending on the specific LLM/VLM. Most testing was performed using `LLaMA` and `LLaVA` models; results with other models may differ. 
* Even small VLMs can exhibit significant processing latency, this might especially increase on mobile hardware. Real-time usage of VLMs is therefore impractical outside of cloud-assisted scenarios. 
* While designed for offline usage, the system can connect to external AI providers using the same `REST` interface. However, prompts must be adjusted to match provider-specific API requirements. (Advanced Blueprint prompt generation for ollama may not align directly with external API specifications and the Blueprint prompt generation can require careful, detail-oriented configuration.) 
* Ollama’s new cloud-model functionality may simplify cloud-based usage in the future. 


