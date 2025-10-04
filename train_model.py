import json
import torch
from datasets import Dataset, DatasetDict
from transformers import (
    DistilBertForSequenceClassification,
    DistilBertTokenizerFast,
    Trainer,
    TrainingArguments,
)

print("🔧 Using CPU (Stable on M1 Mac)")
device = torch.device("cpu")

# Load dataset
with open("dataset.json", "r") as f:
    data = json.load(f)

# Split train/test
train_size = int(len(data) * 0.8)
train_data = data[:train_size]
test_data = data[train_size:]

dataset = DatasetDict(
    {"train": Dataset.from_list(train_data), "test": Dataset.from_list(test_data)}
)

# Create label mappings
labels = sorted(list(set(x["label"] for x in data)))
label2id = {label: i for i, label in enumerate(labels)}
id2label = {i: label for label, i in label2id.items()}

print(f"📊 Dataset Info:")
print(f"   - Training samples: {len(train_data)}")
print(f"   - Test samples: {len(test_data)}")
print(f"   - Labels: {labels}")

def label_to_id(batch):
    batch["label"] = label2id[batch["label"]]
    return batch

dataset = dataset.map(label_to_id)

# Tokenize
tokenizer = DistilBertTokenizerFast.from_pretrained("distilbert-base-uncased")

def tokenize(batch):
    return tokenizer(
        batch["text"], 
        padding="max_length", 
        truncation=True, 
        max_length=128
    )

print("🔧 Tokenizing dataset...")
dataset = dataset.map(tokenize, batched=True)
dataset.set_format("torch", columns=["input_ids", "attention_mask", "label"])

# Model
print("🤖 Loading model...")
model = DistilBertForSequenceClassification.from_pretrained(
    "distilbert-base-uncased",
    num_labels=len(labels),
    id2label=id2label,
    label2id=label2id,
)

# CPU-optimized training arguments
training_args = TrainingArguments(
    output_dir="./results",
    evaluation_strategy="epoch",
    save_strategy="epoch",
    learning_rate=2e-5,
    per_device_train_batch_size=4,  # Small batch size for CPU
    per_device_eval_batch_size=4,
    num_train_epochs=10,
    weight_decay=0.01,
    save_total_limit=2,
    load_best_model_at_end=True,
    metric_for_best_model="eval_loss",
    report_to=None,
    no_cuda=True,  # Ensure no CUDA is used
    dataloader_pin_memory=False,  # Disable pin memory
)

model.to(device)
print(f"✅ Model moved to {device}")

trainer = Trainer(
    model=model,
    args=training_args,
    train_dataset=dataset["train"],
    eval_dataset=dataset["test"],
)

print("🎯 Starting training...")
trainer.train()

print("💾 Saving model...")
trainer.save_model("./intent_model")
tokenizer.save_pretrained("./intent_model")

print("✅ Training complete! Model saved to ./intent_model")