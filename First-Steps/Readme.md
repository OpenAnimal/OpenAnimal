# OpenBrain — Getting Started  

## First Steps

### ✅ Prerequisites
- Download the ONNX networks into: `Plugins/OpenAnimal/Content/OnnxNetworks` *(Create the folder if it does not exist.)*  
- Alternatively, run the **OnnxDownloader** widget located in the `Utils` folder.

---

## Setup Instructions

### 1. Open the OpenBrain Blueprint
Navigate to the `OpenBrain` folder and open the **OpenBrain** Blueprint.

### 2. Copy the Brain Template Into Any Actor
Inside OpenBrain, near **BeginPlay**, you’ll find a blue comment box containing the Brain template.

- Copy these nodes into any Actor that has a **Static Mesh** or **Skeletal Mesh**.
- Right-click the boolean inputs and **create variables** for them.  
These booleans control whether the Brain starts within that Actor.

### 3. Add the Camera Anchor
- Copy the **CameraAnchor** component from `OpenBrain_BP`.
- Attach it to the Parent Actor as a **child of the Static or Skeletal Mesh**.
- Position it where desired.

**Optional (Skeletal Mesh only):**  
Set the parent socket to a head bone (like `"head"`), so the camera follows head movement.

---

## 📝 Additional Notes

- Two example levels with animals are provided.  
See **Penguin_BP** or **Mouse_BP** to understand how Brains attach to Actors.
- Extensive logging is included — check the **Output Log** for edge cases or issues.
- Additional modules (Actor Components) can be added to a Brain:
- LLM / VLM Module  
- WiFi Module  
- Serial Module  
*(Some require external plugins such as **VaRestX**: https://github.com/AboveConstraints/VaRestX)*

---

## ⚠ Special Version Note (UE 5.7)

If you use **UE5.7**, you may need the **VaRestX 5.6** plugin and must manually adjust the plugin version inside the `.uplugin` file.

The plugin will still compile without constant version warnings on startup.

---

## 🎮 Controls

| Action | Input |
|--------|--------|
| Basic controls | Standard movement |
| Set waypoint | **Right-Click** |
| Toggle camera | **C** |
| Switch views | **0-9** or **Tab** |



