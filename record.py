# file: record.py
import sounddevice as sd
import wavio
import requests
import tempfile
import os
import numpy as np
import json

DURATION = 5  # seconds
SAMPLE_RATE = 16000  # Standard for speech recognition
SERVER_URL = "http://127.0.0.1:5000/intent"

def record_voice():
    print(f"🎙️ Recording for {DURATION} seconds... (Speak now)")
    try:
        # Record audio
        audio = sd.rec(
            int(DURATION * SAMPLE_RATE),
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype='int16',
            blocking=True
        )
        print("✅ Recording complete.")
        return audio
    except Exception as e:
        print(f"❌ Recording failed: {e}")
        return None

def save_temp_wav(audio):
    """Save audio to temporary WAV file"""
    try:
        if audio is None or len(audio) == 0:
            raise ValueError("No audio data recorded")
            
        # Remove extra dimensions if present
        audio = np.squeeze(audio)
        
        tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".wav")
        wavio.write(tmp.name, audio, SAMPLE_RATE, sampwidth=2)
        print(f"💾 Audio saved to temporary file")
        return tmp.name
    except Exception as e:
        print(f"❌ Failed to save audio: {e}")
        return None

def send_to_server(filepath):
    """Send audio file to Flask server for processing"""
    if not filepath or not os.path.exists(filepath):
        print("❌ Audio file not found")
        return
        
    print("📤 Sending audio to Flask server...")
    try:
        with open(filepath, "rb") as f:
            files = {"audio": ("recording.wav", f, "audio/wav")}
            resp = requests.post(SERVER_URL, files=files, timeout=60)
        
        print(f"📥 Response status: {resp.status_code}")
        
        if resp.status_code == 200:
            result = resp.json()
            print("✅ Server Response:")
            print(json.dumps(result, indent=2))
        else:
            print(f"❌ Server error (HTTP {resp.status_code}):")
            try:
                error_detail = resp.json()
                print(json.dumps(error_detail, indent=2))
            except:
                print(resp.text)
            
    except requests.exceptions.RequestException as e:
        print(f"❌ Network error: {e}")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")

def test_transcription_only(filepath):
    """Test just the transcription part"""
    if not filepath or not os.path.exists(filepath):
        print("❌ Audio file not found")
        return
        
    print("🎧 Testing transcription only...")
    try:
        with open(filepath, "rb") as f:
            files = {"audio": ("recording.wav", f, "audio/wav")}
            resp = requests.post("http://127.0.0.1:5000/test-transcribe", files=files, timeout=30)
        
        print(f"📥 Response status: {resp.status_code}")
        
        if resp.status_code == 200:
            result = resp.json()
            print("✅ Transcription Result:")
            print(json.dumps(result, indent=2))
            return result.get("transcript", "")
        else:
            print(f"❌ Error (HTTP {resp.status_code}):")
            print(resp.text)
            return None
            
    except requests.exceptions.RequestException as e:
        print(f"❌ Network error: {e}")
        return None

def test_gemini_directly(text):
    """Test Gemini API directly with text"""
    print(f"🧪 Testing Gemini with text: '{text}'")
    try:
        payload = {"text": text}
        resp = requests.post("http://127.0.0.1:5000/test-gemini", json=payload, timeout=30)
        
        print(f"📥 Response status: {resp.status_code}")
        
        if resp.status_code == 200:
            result = resp.json()
            print("✅ Gemini Direct Test Result:")
            print(json.dumps(result, indent=2))
        else:
            print(f"❌ Error (HTTP {resp.status_code}):")
            print(resp.text)
            
    except requests.exceptions.RequestException as e:
        print(f"❌ Network error: {e}")

def check_models():
    """Check available models"""
    print("🔍 Checking available models...")
    try:
        resp = requests.get("http://127.0.0.1:5000/models", timeout=10)
        if resp.status_code == 200:
            result = resp.json()
            print("✅ Available models:")
            for model in result.get("available_models", []):
                print(f"   - {model}")
        else:
            print("❌ Failed to get models")
    except Exception as e:
        print(f"❌ Error checking models: {e}")

def main():
    """Main function to record and process voice"""
    print("=" * 50)
    print("Voice Intent Recognition System (Gemini)")
    print("=" * 50)
    
    # First check available models
    check_models()
    print()
    
    # Record audio
    audio_data = record_voice()
    if audio_data is None:
        return
        
    # Save to temporary file
    temp_file = save_temp_wav(audio_data)
    if not temp_file:
        return
        
    try:
        # First test just transcription
        print("\n1. Testing transcription...")
        transcript = test_transcription_only(temp_file)
        
        if transcript:
            print(f"\n🎯 Detected speech: '{transcript}'")
            
            # Test Gemini directly with the transcript
            print("\n2. Testing Gemini directly with text...")
            test_gemini_directly(transcript)
            
            print("\n3. Testing full intent extraction...")
            # Then test full intent extraction
            send_to_server(temp_file)
        
    finally:
        # Clean up temporary file
        try:
            os.remove(temp_file)
            print(f"🧹 Cleaned up temporary file")
        except:
            pass

if __name__ == "__main__":
    main()