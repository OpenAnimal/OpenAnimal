## Large Language Models and Visual Language Models


The plugin aims to provide different versions of intelligence. While the basic plugin focusses on replicating basic socio-dynamic and 'alive' behavior, a higher kind of cognition and advanced intelligence can be used throughout the plugin. 

Due to the philosophy of the plugin to allow standalone and offline, while also striving for simplicity, we focussed on the use of Ollama as a local provider for language and visual models.

Ollama requires nearly no effort to get and host minimal up to very complex models locally.

Get Ollama here: https://ollama.com/

In addition to text based prompting, we created a Blueprint based prompt creation, which allows to specify structured, f.i. Json based returns. We created a automatic capturing of Types (including Arrays) as returns to automatically set Blueprint variables.

Images can be sent via a prompt, by using utility functions that encode images or render targets from Blueprints to base64 as the common format

Within the OpenAnimal Plugin the LLM/VLM pipeline is in a Actor Component that can be attached to the brain and that includes detailed instructions and Prompt examples. 


The communication is done via the [VaRest](https://github.com/AboveConstraints/VaRestX) Plugin. This is a third party dependency to simply create Http REST connections (based on URLs and requiring only IP, Port and added content). While first developments were made with in-house UE Json and Http solutions, these were yet so instable and nodes were randomly dropped after restarts, we switched to VaRest as a battletested and reliable solution for now.

Drawbacks:
- Each Model can have slightly different variations in prompts or capabilities. Most of our tests were done on LLama and LLava models, but results for other models might differ.
- Even the smallest Visual-Language Models can take a considerable amount of time to process. This would very likely be more pronounced if deployed on mobile phones. (Running Ollama on ARM devices might also not be that straight-forward currently.) Therefor using a VLM for real-time purposes might not be feasable (if not using cloud models, see below), why it is now reserved for higher and deeper analysis cognition.

- OpenAnimal is meant to work offline, but it allows to use the same Http REST interface to access AI provider APIs the same way as it interfaces with Ollama locally. Though the prompts have to be set up differently (providing providers IPs and ones Tokens one received from the provider). This might require some manual work and the advanced Blueprint-based prompt generation might not simply work for the providers API specifications.
However Ollama just released 'cloud models', which might simplify the use of cloud models via Ollama style prompts.
 

