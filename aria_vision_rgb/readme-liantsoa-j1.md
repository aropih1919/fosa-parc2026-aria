# aria_vision_rgb — Package ROS 2 de Vision RGB

> **Projet** : PARC2026 — Robot ARIA  
> **Auteur** : Liantsoa (Liantsoa-and)  
> **Branche Git** : `feature/vision-rgb`  
> **Plateforme** : The Construct — ROS 2 Jazzy — Gazebo Harmonic  
> **Date J1** : 01 août 2026

---

## Table des matières

1. [Contexte du projet ARIA](#1-contexte-du-projet-aria)
2. [Rôle de Liantsoa dans l'équipe](#2-rôle-de-liantsoa-dans-léquipe)
3. [Définitions techniques essentielles](#3-définitions-techniques-essentielles)
4. [Architecture globale du système](#4-architecture-globale-du-système)
5. [Description du package aria_vision_rgb](#5-description-du-package-aria_vision_rgb)
6. [Le nœud rgb_vision_node — fonctionnement détaillé](#6-le-nœud-rgb_vision_node--fonctionnement-détaillé)
7. [Détection par seuillage HSV — explication](#7-détection-par-seuillage-hsv--explication)
8. [Topics ROS 2 publiés et souscrits](#8-topics-ros-2-publiés-et-souscrits)
9. [Environnement de développement mis en place](#9-environnement-de-développement-mis-en-place)
10. [Étapes de mise en place complètes (J1)](#10-étapes-de-mise-en-place-complètes-j1)
11. [Structure du package](#11-structure-du-package)
12. [Commandes utiles](#12-commandes-utiles)
13. [Planning et livrables](#13-planning-et-livrables)
14. [Interfaces partagées (aria_msgs)](#14-interfaces-partagées-aria_msgs)

---

## 1. Contexte du projet ARIA

Le projet **ARIA** (Autonomous Robot for Intelligent Action) est le projet d'équipe préparé pour la compétition **PARC2026** (Pan-African Robotics Competition 2026). Il s'agit d'un robot autonome capable de :

1. Recevoir une **commande texte** simulant une commande vocale (ex : *"Va vers la zone nord et trouve le baril bleu"*)
2. **Naviguer** vers une zone géographique via GPS et Nav2 en évitant les obstacles au LiDAR
3. **Détecter** un objet cible par vision (caméra RGB et caméra ZED 2i stéréo)
4. **Publier un rapport de mission** contenant la position de l'objet, la distance et le niveau de confiance

**Plateforme d'exécution** : [The Construct](https://app.theconstruct.ai) — environnement cloud qui fournit ROS 2 Jazzy et Gazebo Harmonic sans installation locale lourde.

**Dépôt GitHub** : `https://github.com/aropih1919/fosa-parc2026-aria`

---

## 2. Rôle de Liantsoa dans l'équipe

| Membre | Domaine | Package |
|--------|---------|---------|
| Faneva | Caméra ZED 2i — perception 3D, fusion 2D+3D | `aria_vision_zed` |
| **Liantsoa** | **Caméra RGB + Computer Vision — détection couleur/forme, bounding box** | **`aria_vision_rgb`** |
| Edinah | GPS & Navigation (Nav2, waypoints) | `aria_gps_nav` |
| Jeannie | NLP — compréhension de la commande texte | `aria_nlp_stt` |
| Mpiaro | NLP — orchestration (state machine + action serveur) | `aria_nlp_mission` |

**Binôme clé** : Liantsoa ↔ Faneva pour la **fusion vision 2D + profondeur 3D** (J3).

Liantsoa est responsable de la **détection visuelle par caméra RGB ordinaire** : identifier la couleur et la forme des objets dans l'image, et publier leurs coordonnées en pixels sous forme de bounding boxes. Ce travail alimente ensuite Faneva qui ajoute la dimension profondeur (ZED 2i) pour obtenir une position 3D réelle dans l'espace.

---

## 3. Définitions techniques essentielles

### ROS 2 (Robot Operating System 2)
Framework open-source pour la programmation de robots. Il permet à des **nœuds** (programmes) de communiquer entre eux via des **topics** (canaux de messages). ROS 2 Jazzy est la version LTS (Long Term Support) stable utilisée pour ce projet.

### Nœud ROS 2
Un nœud est un **programme indépendant** qui s'exécute dans le système ROS. Chaque nœud a une responsabilité précise. `rgb_vision_node` est le nœud de détection visuelle RGB. Les nœuds communiquent par publication/souscription sur des topics.

### Topic ROS 2
Canal de communication **publish/subscribe** entre nœuds. Un nœud publie des messages sur un topic, d'autres nœuds s'y abonnent pour recevoir ces messages. Exemple : `rgb_vision_node` souscrit à `/camera/image_raw` (images entrantes) et publie sur `/vision/detections_2d` (détections).

### Gazebo Harmonic
Simulateur robotique 3D open-source. Il simule l'environnement physique (monde, obstacles, lumière), le robot (moteurs, capteurs) et les capteurs (caméra, LiDAR, GPS) sans avoir besoin d'un vrai robot physique.

### colcon
Outil de **build** (compilation) pour ROS 2. Il compile tous les packages d'un workspace ROS 2 et génère les fichiers exécutables. Commande principale : `colcon build`.

### Workspace ROS 2
Répertoire de travail organisé pour ROS 2, structuré en `src/` (sources), `build/` (compilation), `install/` (binaires), `log/` (journaux). Sur The Construct, il est situé dans `~/ros2_ws/`.

### Package ROS 2
Unité modulaire de code ROS 2. Chaque fonctionnalité du robot est dans un package séparé. Un package Python ROS 2 contient : `package.xml` (métadonnées et dépendances), `setup.py` (configuration d'installation), et les fichiers Python du code.

### OpenCV (cv2)
Bibliothèque de **Computer Vision** (vision par ordinateur) open-source. Elle fournit des fonctions pour traiter des images : conversion de couleurs, filtrage, détection de contours, etc. Utilisée ici pour la détection HSV.

### cv_bridge
Pont ROS 2 ↔ OpenCV. Convertit les messages `sensor_msgs/Image` (format ROS) en images NumPy/OpenCV manipulables, et vice versa.

### HSV (Hue, Saturation, Value)
Espace de couleur alternatif au RGB, plus adapté à la détection de couleurs sous différentes conditions d'éclairage. **H** (teinte) : couleur pure (rouge, bleu, jaune…), **S** (saturation) : intensité de la couleur, **V** (valeur) : luminosité. Le seuillage HSV (`cv2.inRange`) isole une couleur précise même si l'éclairage change.

### Bounding Box
Rectangle englobant détecté autour d'un objet dans une image. Défini par sa position centre `(cx, cy)` et ses dimensions `(largeur, hauteur)` en pixels. Utilisé pour localiser visuellement l'objet détecté.

### Detection2DArray (vision_msgs)
Message ROS 2 standardisé pour transmettre une liste de détections 2D. Chaque `Detection2D` contient une bounding box et une liste d'hypothèses (classe de l'objet + score de confiance).

### The Construct / ROSject
The Construct est une **plateforme cloud** pour le développement robotique ROS. Un **ROSject** est un environnement de travail cloud préconfiguré avec ROS 2, Gazebo, et un workspace prêt à l'emploi, accessible depuis n'importe quel navigateur web.

### Personal Access Token (GitHub)
Jeton d'authentification GitHub remplaçant le mot de passe pour les opérations Git en ligne de commande (push, pull sur repos privés). Généré depuis GitHub → Settings → Developer Settings → Tokens.

---

## 4. Architecture globale du système

```
commande texte → [NLP: Jeannie] → intent → [Mission Manager: Mpiaro]
                                                │
                          ┌─────────────────────┼─────────────────────┐
                          ▼                     ▼                     ▼
               [GPS/Nav: Edinah]    [Vision RGB/CV: Liantsoa]   [ZED: Faneva]
                                              │
                   LiDAR (réutilisé)          └──────┬──────────────────┘
                                                     ▼
                                             rapport de mission
```

**Interfaces partagées** (package `aria_msgs`, figées J1) :
- `NLPIntent` — intention parsée depuis la commande texte
- `Detection2D` — détection visuelle 2D (bounding box)
- `MissionReport` — rapport final de mission
- Action `SearchAndReport` — goal/result/feedback pour la mission complète

---

## 5. Description du package aria_vision_rgb

`aria_vision_rgb` est le package ROS 2 Python dédié à la **détection visuelle par caméra RGB**. Son rôle dans le pipeline ARIA est de :

1. Recevoir le flux d'images brutes de la caméra RGB (`/camera/image_raw`)
2. Analyser chaque image pour détecter des objets par leur **couleur** (seuillage HSV) et leur **forme** (contours)
3. Publier les détections sous forme de **bounding boxes** sur `/vision/detections_2d`
4. Fournir ces données à Faneva (ZED) pour la fusion 2D+3D → position réelle de l'objet

**Dépendances déclarées dans `package.xml`** :
- `rclpy` — API Python ROS 2
- `sensor_msgs` — pour le type `Image`
- `cv_bridge` — conversion ROS ↔ OpenCV
- `vision_msgs` — pour `Detection2DArray`, `Detection2D`, `ObjectHypothesisWithPose`

---

## 6. Le nœud rgb_vision_node — fonctionnement détaillé

Le nœud `rgb_vision_node` (fichier `rgb_vision_node.py`) est la pièce centrale du package.

### Initialisation
À son démarrage, le nœud :
- Crée un **subscriber** sur `/camera/image_raw` (images RGB entrantes)
- Crée un **publisher** sur `/vision/detections_2d` (détections à publier)
- Instancie `CvBridge` pour convertir les images ROS en format OpenCV

### Traitement par image (callback)
À chaque image reçue, le nœud :
1. Convertit l'image ROS en tableau NumPy BGR via `cv_bridge`
2. Convertit BGR → HSV via `cv2.cvtColor`
3. Pour chaque couleur cible (bleu, rouge, jaune) :
   - Applique un masque binaire `cv2.inRange(hsv, lower, upper)`
   - Trouve les contours dans le masque avec `cv2.findContours`
   - Filtre les contours trop petits (aire < 500 pixels)
   - Calcule la bounding box `(x, y, w, h)` de chaque contour valide
   - Calcule le score de confiance = aire du contour / aire totale de l'image
   - Construit un message `Detection2D` avec la bounding box et l'hypothèse
4. Publie le `Detection2DArray` complet

### Couleurs détectées (J1)

| Couleur | H min | H max | S min | V min |
|---------|-------|-------|-------|-------|
| Bleu | 100 | 130 | 150 | 50 |
| Rouge | 0 | 10 | 120 | 70 |
| Jaune | 20 | 35 | 100 | 100 |

---

## 7. Détection par seuillage HSV — explication

L'espace HSV est utilisé car il **sépare la couleur (H) de la luminosité (V)**, ce qui rend la détection robuste aux variations d'éclairage contrairement au RGB brut.

```
Image BGR (caméra)
      │
      ▼
cv2.cvtColor(BGR→HSV)
      │
      ▼
cv2.inRange(hsv, [H_min, S_min, V_min], [H_max, S_max, V_max])
      │
      ▼
Masque binaire (blanc = couleur détectée, noir = reste)
      │
      ▼
cv2.findContours → liste de contours
      │
      ▼
Filtrage (aire > 500px) → Bounding box (x, y, w, h)
      │
      ▼
Publication Detection2DArray sur /vision/detections_2d
```

---

## 8. Topics ROS 2 publiés et souscrits

| Direction | Topic | Type de message | Description |
|-----------|-------|-----------------|-------------|
| **Souscrit** | `/camera/image_raw` | `sensor_msgs/Image` | Flux images caméra RGB |
| **Publié** | `/vision/detections_2d` | `vision_msgs/Detection2DArray` | Bounding boxes + classes détectées |

Vérification en terminal :
```bash
ros2 topic list          # liste tous les topics actifs
ros2 topic echo /vision/detections_2d   # affiche les détections en temps réel
ros2 node list           # vérifie que /rgb_vision_node est actif
```

---

## 9. Environnement de développement mis en place

### Plateforme
- **The Construct** (cloud) — ROSject `parc2026-aria`
- ROS 2 Jazzy + Gazebo Harmonic (préinstallés sur le ROSject)
- Workspace : `~/ros2_ws/`

### Pourquoi The Construct plutôt qu'une installation locale ?
Ubuntu 26.04 LTS (système de Liantsoa) n'est pas encore officiellement supporté par ROS 2 Jazzy et Gazebo Harmonic dont les binaires ciblent Ubuntu 24.04. The Construct évite ce problème en fournissant un environnement Ubuntu 24.04 préconfiguré dans le cloud.

### Git
- Dépôt : `https://github.com/aropih1919/fosa-parc2026-aria`
- Authentification : Personal Access Token GitHub (HTTPS)
- Branche de travail : `feature/vision-rgb`
- Convention de commits : `feat:`, `fix:`, `docs:`, `test:`, `refactor:`

---

## 10. Étapes de mise en place complètes (J1)

Voici le récit complet de tout ce qui a été fait le J1 pour mettre en place l'environnement et produire le livrable.

### 1. Création et lancement du ROSject
Création d'un nouveau ROSject `parc2026-aria` sur The Construct avec la configuration ROS 2 Jazzy + Ubuntu 24.04. Le ROSject fournit un workspace `~/ros2_ws/` prêt à l'emploi avec deux terminaux intégrés et un éditeur de code.

### 2. Clone du dépôt GitHub dans le ROSject
```bash
cd ~/ros2_ws/src
git clone https://github.com/aropih1919/fosa-parc2026-aria
# → dossier créé : fosa-parc2026-aria/
# Contenu initial : aria_bringup/, aria_description/, aria_msgs/
```

### 3. Création de la branche feature/vision-rgb
La branche n'existait pas encore sur le remote. Elle a été créée localement puis poussée :
```bash
cd ~/ros2_ws/src/fosa-parc2026-aria
git checkout -b feature/vision-rgb
# Configuration du token pour l'authentification HTTPS
git remote set-url origin https://Liantsoa-and:<TOKEN>@github.com/aropih1919/fosa-parc2026-aria.git
git push -u origin feature/vision-rgb
```

### 4. Création du package aria_vision_rgb
```bash
cd ~/ros2_ws/src/fosa-parc2026-aria
ros2 pkg create aria_vision_rgb \
  --build-type ament_python \
  --dependencies rclpy sensor_msgs cv_bridge vision_msgs
```
Cette commande génère automatiquement la structure standard d'un package Python ROS 2 : `package.xml`, `setup.py`, `setup.cfg`, et le dossier Python du même nom.

### 5. Écriture du nœud rgb_vision_node.py
Création de `aria_vision_rgb/aria_vision_rgb/rgb_vision_node.py` avec :
- Subscriber sur `/camera/image_raw`
- Détection HSV pour 3 couleurs (bleu, rouge, jaune)
- Filtrage par aire minimale (500 px²)
- Publication `Detection2DArray` sur `/vision/detections_2d`

### 6. Déclaration du point d'entrée dans setup.py
```python
'console_scripts': [
    'rgb_vision_node = aria_vision_rgb.rgb_vision_node:main',
],
```

### 7. Build et test
```bash
cd ~/ros2_ws
colcon build --packages-select aria_vision_rgb
source install/setup.bash
ros2 run aria_vision_rgb rgb_vision_node
# → [INFO] rgb_vision_node démarré — en attente d image...
```
Vérification des topics actifs : `/vision/detections_2d` présent → **J1 validé**.

### 8. Commit et push
```bash
git config --global user.email "liantsoaandreane@gmail.com"
git config --global user.name "Liantsoa-and"
git add aria_vision_rgb/
git commit -m "feat: aria_vision_rgb — rgb_vision_node détection HSV multi-couleurs (J1)"
git push origin feature/vision-rgb
```

---

## 11. Structure du package

```
fosa-parc2026-aria/
├── aria_bringup/          # launch files, mondes Gazebo, config
├── aria_description/      # URDF du robot (base + capteurs)
├── aria_msgs/             # interfaces partagées (figées J1)
└── aria_vision_rgb/       # ← package de Liantsoa
    ├── aria_vision_rgb/
    │   ├── __init__.py
    │   └── rgb_vision_node.py    # nœud principal de détection
    ├── package.xml               # métadonnées et dépendances ROS 2
    ├── setup.py                  # point d'entrée console_scripts
    └── setup.cfg
```

---

## 12. Commandes utiles

```bash
# Builder uniquement ce package
colcon build --packages-select aria_vision_rgb

# Sourcer le workspace après build
source ~/ros2_ws/install/setup.bash

# Lancer le nœud
ros2 run aria_vision_rgb rgb_vision_node

# Vérifier que le nœud est actif
ros2 node list

# Vérifier les topics publiés
ros2 topic list

# Lire les détections en temps réel
ros2 topic echo /vision/detections_2d

# Workflow Git quotidien
git pull                          # récupérer le travail de l'équipe
git add aria_vision_rgb/
git commit -m "feat: description"
git push origin feature/vision-rgb
```

---

## 13. Planning et livrables

| Jour | Objectif Liantsoa | Statut |
|------|-------------------|--------|
| **J1** | Détection couleur simple (seuillage HSV) sur image fixe, nœud minimal qui tourne | ✅ Fait |
| **J2** | Détection robuste multi-couleurs/formes, bounding boxes fiables, filtrage morphologique | 🔜 |
| **J3** | Fusion Detection2D + profondeur ZED → `/vision/target_pose` (avec Faneva) | 🔜 |
| **J4** | Intégration chaîne complète, gestion des cas d'échec (objet non trouvé) | 🔜 |
| **J5** | Stabilisation, documentation, démo finale enregistrée | 🔜 |

**Critère de validation RGB/CV** : détection réussie sur 8 images sur 10 avec bounding box correcte.

---

## 14. Interfaces partagées (aria_msgs)

Les messages partagés entre les membres de l'équipe sont définis dans `aria_msgs/` et **figés depuis J1**. Toute modification après J1 doit être annoncée à l'équipe avant push.

| Message / Action | Champs principaux | Utilisé par |
|-----------------|-------------------|-------------|
| `NLPIntent` | `target_color`, `target_zone`, `action` | Jeannie → Mpiaro |
| `Detection2D` | `bbox`, `class_id`, `score` | Liantsoa → Faneva |
| `MissionReport` | `position`, `distance`, `confidence` | Mpiaro (sortie finale) |
| `SearchAndReport` (action) | `goal`, `result`, `feedback` | Mpiaro (orchestration) |

---

*Dernière mise à jour : J1 — 01 août 2026*  
*Auteur : Liantsoa-and — Package : aria_vision_rgb — Branche : feature/vision-rgb*