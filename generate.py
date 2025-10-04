import json
import random

# COCO object classes (1–80)
coco_objects = [
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "dining table",
    "toilet",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush",
]

# actions the robot can perform on objects
actions = {
    "goto": [
        "go to the {obj}",
        "move to the {obj}",
        "go near the {obj}",
        "pick up the {obj}",
        "grab the {obj}",
        "take the {obj}",
        "take me to the {obj}",
        "navigate to the {obj}",
    ],
    "follow": [
        "follow the {obj}",
        "come after the {obj}",
        "walk behind the {obj}",
        "trail the {obj}",
        "shadow the {obj}",
        "pursue the {obj}",
        "chase the {obj}",
    ],
    "look": [
        "look at the {obj}",
        "focus on the {obj}",
        "observe the {obj}",
        "check the {obj}",
        "inspect the {obj}",
        "examine the {obj}",
        "view the {obj}",
        "watch the {obj}",
    ],
}

# special commands that don't require a specific object
special_intents = {
    "STOP": [
        "stop",
        "halt",
        "wait here",
        "pause",
        "cease movement",
        "hold position",
        "stay",
        "freeze",
        "stand still",
        "don't move",
        "stop now",
        "stop immediately",
    ],
    "TURN_LEFT": [
        "turn left",
        "go left",
        "rotate left",
        "veer left",
        "head left",
        "shift left",
        "move left",
        "head to the left",
        "turn to the left",
    ],
    "TURN_RIGHT": [
        "turn right",
        "go right",
        "rotate right",
        "veer right",
        "head right",
        "shift right",
        "move right",
        "head to the right",
        "turn to the right",
    ],
    "GO_TO_DOCK": [
        "go to your dock",
        "return to charger",
        "recharge",
        "dock now",
        "go home",
        "head to dock",
        "dock",
        "return home",
        "head home",
    ],
    "TURN_AROUND": [
        "turn around",
        "do a 180",
        "turn completely around",
        "reverse direction",
        "face the opposite way",
        "turn back",
        "rotate 180 degrees",
        "spin around",
    ],
    "SLOW_DOWN": [
        "slow down",
        "move slower",
        "decrease speed",
        "reduce speed",
        "ease up",
        "take it easy",
        "go slower",
        "reduce pace",
    ],
    "SPEED_UP": [
        "speed up",
        "move faster",
        "increase speed",
        "go faster",
        "hurry up",
        "accelerate",
        "pick up the pace",
        "step on it",
        "go quickly",
    ],
    "MOVE_BACK": [
        "go back",
        "move backward",
        "reverse",
        "back up",
        "retreat",
        "go in reverse",
        "move to the back",
        "step back",
        "back away",
    ],
    "MOVE_FORWARD": [
        "go forward",
        "move ahead",
        "proceed",
        "advance",
        "move forward",
        "head forward",
        "continue ahead",
        "press on",
        "go straight ahead",
    ],
}

dataset = []

# Generate action-object intents
for obj in coco_objects:
    for action, templates in actions.items():
        label = f"{action.upper()}_{obj.replace(' ', '_').upper()}"
        for template in templates:
            text = template.format(obj=obj)
            dataset.append({"text": text, "label": label})

# Add special intents
for label, phrases in special_intents.items():
    for phrase in phrases:
        dataset.append({"text": phrase, "label": label})

# Shuffle dataset
random.shuffle(dataset)

# Save to JSON
with open("coco_intent_dataset.json", "w") as f:
    json.dump(dataset, f, indent=2)

print(f"Generated dataset with {len(dataset)} samples: coco_intent_dataset.json")
