# file: gemini.py
import os
import json
from flask import Flask, request, jsonify
from flask_cors import CORS
import speech_recognition as sr
import tempfile
import google.generativeai as genai

app = Flask(__name__)
CORS(app)

# Get API key from environment variable
GEMINI_KEY = os.getenv("GEMINI_API_KEY")
if not GEMINI_KEY:
    raise RuntimeError("Set GEMINI_API_KEY environment variable first!")

# Configure Gemini
genai.configure(api_key=GEMINI_KEY)

def get_available_models():
    """List available Gemini models"""
    try:
        models = genai.list_models()
        available_models = []
        for model in models:
            if 'generateContent' in model.supported_generation_methods:
                available_models.append(model.name)
        return available_models
    except Exception as e:
        print(f"Error listing models: {e}")
        return []

def transcribe_audio_local(audio_file):
    """
    Transcribe audio using local speech recognition
    """
    recognizer = sr.Recognizer()
    
    try:
        # Save the uploaded file to a temporary file
        with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as temp_file:
            audio_file.save(temp_file.name)
            temp_path = temp_file.name

        # Use the temporary file with speech_recognition
        with sr.AudioFile(temp_path) as source:
            # Adjust for ambient noise and record the audio
            print("🔊 Adjusting for ambient noise...")
            recognizer.adjust_for_ambient_noise(source, duration=0.5)
            audio_data = recognizer.record(source)
            
            print("🎤 Transcribing audio...")
            # Use Google's speech recognition (free, no API key needed for small amounts)
            text = recognizer.recognize_google(audio_data)
            
        # Clean up temporary file
        os.unlink(temp_path)
        return text.strip()
        
    except sr.UnknownValueError:
        raise Exception("Could not understand the audio")
    except sr.RequestError as e:
        raise Exception(f"Speech recognition error: {e}")
    except Exception as e:
        raise Exception(f"Audio processing error: {e}")

def extract_intent_gemini(transcript):
    """
    Extract intent using Google Gemini
    """
    try:
        # Use the available models from your list
        model_names = [
            "models/gemini-2.0-flash",  # Fast and free
            "models/gemini-2.0-flash-001",  # Specific version
            "models/gemini-2.0-flash-lite",  # Lightweight
            "models/gemini-2.0-flash-lite-001",  # Lite version
            "models/gemini-2.0-pro-exp",  # Pro experimental
            "models/gemini-flash-latest",  # Latest flash
            "models/gemini-pro-latest",  # Latest pro
            "models/gemma-3-4b-it",  # Gemma model
            "models/gemma-3-12b-it",  # Larger Gemma
        ]
        
        system_prompt = """You are an intent-extraction assistant. 
Given a user utterance, return STRICTLY valid JSON with this exact format:
{"intent": "string", "confidence": number between 0-1, "entities": {"key1": "value1", "key2": "value2"}}

Available intents: 
- "play_music" (when user wants to play songs or music)
- "weather_query" (when asking about weather)  
- "timer_set" (when setting timers or reminders)
- "question" (when asking general questions)
- "information" (when seeking specific information)
- "greeting" (when saying hello, hi, etc.)
- "goodbye" (when saying bye, goodbye, etc.)
- "unknown" (when intent is unclear)

Extract entities like:
- For music: song_name, artist, genre
- For weather: location, time, date
- For timer: duration, purpose
- For questions: subject, type

Examples:
Input: "play some jazz music"
Output: {"intent": "play_music", "confidence": 0.9, "entities": {"genre": "jazz"}}

Input: "what's the weather in Tokyo tomorrow"
Output: {"intent": "weather_query", "confidence": 0.95, "entities": {"location": "Tokyo", "time": "tomorrow"}}

Input: "set a timer for 5 minutes"
Output: {"intent": "timer_set", "confidence": 0.98, "entities": {"duration": "5 minutes"}}

IMPORTANT: Return ONLY the JSON, no additional text or explanations."""

        prompt = f"{system_prompt}\n\nExtract intent from this transcript: \"{transcript}\""

        print("🤖 Sending to Gemini for intent extraction...")
        
        # Try each model until one works
        last_error = None
        for model_name in model_names:
            try:
                print(f"  Trying model: {model_name}")
                model = genai.GenerativeModel(model_name)
                response = model.generate_content(prompt)
                
                if not response.text:
                    continue
                    
                # Clean the response - remove markdown code blocks if present
                assistant_text = response.text.strip()
                assistant_text = assistant_text.replace('```json', '').replace('```', '').strip()

                print(f"📄 Raw Gemini response: {assistant_text}")
                
                try:
                    intent_result = json.loads(assistant_text)
                    print(f"✅ Success with model: {model_name}")
                    return intent_result
                except json.JSONDecodeError as e:
                    print(f"JSON parse error with {model_name}: {e}")
                    # Try to extract JSON from the response if it's wrapped in text
                    import re
                    json_match = re.search(r'\{.*\}', assistant_text, re.DOTALL)
                    if json_match:
                        try:
                            intent_result = json.loads(json_match.group())
                            print(f"✅ Success with model: {model_name} (extracted JSON)")
                            return intent_result
                        except:
                            continue
                    continue
                    
            except Exception as e:
                last_error = e
                print(f"  Model {model_name} failed: {e}")
                continue
        
        # If all models fail, raise the last error
        raise Exception(f"All models failed. Last error: {last_error}")
            
    except Exception as e:
        print(f"Gemini API error: {e}")
        raise

@app.route("/intent", methods=["POST"])
def get_intent():
    """
    Accepts multipart/form-data with an 'audio' file.
    Transcribes it locally and extracts intent with Gemini.
    """
    if "audio" not in request.files:
        return jsonify({"error": "No audio file uploaded"}), 400

    audio = request.files["audio"]
    
    if audio.filename == '':
        return jsonify({"error": "No selected file"}), 400

    try:
        # Step 1: Transcribe locally
        print("🎧 Starting local transcription...")
        transcript = transcribe_audio_local(audio)
        
        if not transcript:
            return jsonify({"error": "No transcript generated"}), 500

        print(f"🎧 Transcript: {transcript}")

        # Step 2: Extract intent using Gemini
        intent_result = extract_intent_gemini(transcript)

        return jsonify({
            "transcript": transcript,
            "intent_result": intent_result,
            "status": "success"
        })

    except Exception as e:
        print(f"Unexpected error: {e}")
        return jsonify({"error": "Internal server error", "detail": str(e)}), 500

@app.route("/health", methods=["GET"])
def health_check():
    """Health check endpoint"""
    available_models = get_available_models()
    return jsonify({
        "status": "healthy", 
        "service": "voice_intent",
        "gemini_configured": GEMINI_KEY is not None,
        "available_models": available_models
    })

@app.route("/test-transcribe", methods=["POST"])
def test_transcribe():
    """Test endpoint for transcription only"""
    if "audio" not in request.files:
        return jsonify({"error": "No audio file uploaded"}), 400

    audio = request.files["audio"]
    
    try:
        transcript = transcribe_audio_local(audio)
        return jsonify({
            "transcript": transcript,
            "status": "success"
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route("/test-gemini", methods=["POST"])
def test_gemini():
    """Test Gemini API directly with text"""
    data = request.get_json()
    if not data or 'text' not in data:
        return jsonify({"error": "No text provided"}), 400
    
    try:
        intent_result = extract_intent_gemini(data['text'])
        return jsonify({
            "text": data['text'],
            "intent_result": intent_result,
            "status": "success"
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route("/models", methods=["GET"])
def list_models():
    """List available Gemini models"""
    try:
        models = get_available_models()
        return jsonify({
            "available_models": models,
            "status": "success"
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == "__main__":
    # Print available models on startup
    print("🔍 Checking available Gemini models...")
    available_models = get_available_models()
    if available_models:
        print("✅ Available models:")
        for model in available_models:
            print(f"   - {model}")
    else:
        print("❌ No available models found or API key issue")
    
    app.run(port=5000, debug=True)