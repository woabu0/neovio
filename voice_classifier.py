import sounddevice as sd
import vosk
import sys
import queue
import json
from transformers import pipeline

# Object Synonym Mapping
object_synonym_map = {
    # People and Animals
    "human": "person",
    "child": "person",
    "kid": "person",
    "infant": "person",
    "baby": "person",
    "adult": "person",
    "man": "person",
    "woman": "person",
    "doggy": "dog",
    "puppy": "dog",
    "canine": "dog",
    "pup": "dog",
    "kitty": "cat",
    "kitten": "cat",
    "feline": "cat",
    "horseback": "horse",
    "sheepdog": "sheep",
    "cowboy": "cow",
    "zombie": "person",  # common in some datasets
    "bear cub": "bear",
    "giraffee": "giraffe",  # common misspelling
    # Vehicles and Transportation
    "auto": "car",
    "automobile": "car",
    "vehicle": "car",
    "truck": "truck",
    "lorry": "truck",
    "bike": "bicycle",
    "cycle": "bicycle",
    "motorcycle": "motorcycle",
    "motorbike": "motorcycle",
    "plane": "airplane",
    "jet": "airplane",
    "bus": "bus",
    "boat": "boat",
    "ship": "boat",
    "train": "train",
    "choo-choo": "train",
    "tram": "train",
    "scooter": "motorcycle",
    # Street and Outdoor Items
    "light": "traffic light",
    "hydrant": "fire hydrant",
    "sign": "stop sign",
    "bench": "bench",
    "parking meter": "parking meter",
    "tree": "potted plant",
    "shrub": "potted plant",
    # Sports and Recreation
    "ball": "sports ball",
    "football": "sports ball",
    "soccer ball": "sports ball",
    "basketball": "sports ball",
    "baseball": "sports ball",
    "tennis ball": "sports ball",
    "bat": "baseball bat",
    "glove": "baseball glove",
    "board": "skateboard",
    "surfboard": "surfboard",
    "racket": "tennis racket",
    "frisbee": "frisbee",
    "ski": "skis",
    "snowboard": "snowboard",
    "kite": "kite",
    # Household and Furniture
    "couch": "couch",
    "sofa": "couch",
    "chair": "chair",
    "stool": "chair",
    "armchair": "chair",
    "desk": "chair",  # can be used in some contexts
    "table": "dining table",
    "bed": "bed",
    "plant": "potted plant",
    "toilet": "toilet",
    "sink": "sink",
    "fridge": "refrigerator",
    "icebox": "refrigerator",
    "oven": "oven",
    "microwave": "microwave",
    "toaster": "toaster",
    "book": "book",
    "paperback": "book",
    "clock": "clock",
    "watch": "clock",  # common synonym
    "vase": "vase",
    "scissors": "scissors",
    "teddy bear": "teddy bear",
    "bear": "teddy bear",
    "hair drier": "hair drier",
    "dryer": "hair drier",
    "toothbrush": "toothbrush",
    "vessel": "vase",
    # Electronics and Office
    "tv": "tv",
    "television": "tv",
    "screen": "tv",
    "monitor": "tv",
    "laptop": "laptop",
    "computer": "laptop",
    "pc": "laptop",
    "notebook": "laptop",
    "mouse": "mouse",
    "keyboard": "keyboard",
    "phone": "cell phone",
    "mobile": "cell phone",
    "cellphone": "cell phone",
    "remote": "remote",
    "controller": "remote",
    # Containers and Food
    "backpack": "backpack",
    "bag": "backpack",
    "handbag": "handbag",
    "purse": "handbag",
    "suitcase": "suitcase",
    "luggage": "suitcase",
    "umbrella": "umbrella",
    "bottle": "bottle",
    "cup": "cup",
    "mug": "cup",
    "bowl": "bowl",
    "plate": "bowl",  # can be a flat bowl
    "glass": "wine glass",
    "wineglass": "wine glass",
    "fork": "fork",
    "knife": "knife",
    "spoon": "spoon",
    "banana": "banana",
    "apple": "apple",
    "sandwich": "sandwich",
    "orange": "orange",
    "broccoli": "broccoli",
    "carrot": "carrot",
    "hot dog": "hot dog",
    "pizza": "pizza",
    "donut": "donut",
    "cake": "cake",
}


def preprocess_text(text, synonym_map):
    lower_text = text.lower()
    for synonym, canonical in synonym_map.items():
        if f" {synonym}" in f" {lower_text}":
            return text.replace(synonym, canonical, 1)
    return text


# Load models
model = vosk.Model("vosk-model")
classifier = pipeline(
    "text-classification", model="./intent_model", device=0 if vosk.Model else -1
)

# Audio setup
q = queue.Queue()


def callback(indata, frames, time, status):
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))


# Start listening
with sd.RawInputStream(
    samplerate=16000,
    blocksize=8000,
    dtype="int16",
    channels=1,
    callback=callback,
):
    rec = vosk.KaldiRecognizer(model, 16000)
    print("Listening...")

    while True:
        data = q.get()
        if rec.AcceptWaveform(data):
            result = json.loads(rec.Result())
            text = result["text"]

            if text.strip():
                # Preprocess text before classification
                processed_text = preprocess_text(text, object_synonym_map)
                prediction = classifier(processed_text)
                intent = prediction[0]["label"]
                confidence = prediction[0]["score"]

                print(f"Original: {text}")
                if text != processed_text:
                    print(f"Processed: {processed_text}")
                print(f"Intent: {intent} ({confidence:.3f})")
                print("-" * 40)
