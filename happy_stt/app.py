from flask import Flask, jsonify
import sounddevice as sd
import soundfile as sf
import numpy as np
import whisper
import torch
import simpleaudio as sa
import logging

app = Flask(__name__)
app.logger.setLevel(logging.INFO)

device = "cuda" if torch.cuda.is_available() else "cpu"
app.logger.info(f"🚀 使用デバイス: {device}")

model = whisper.load_model("small").to(device)

TARGET_MIC_NAME = "USB PnP Sound Device"
DEVICE_INDEX = None
for i, d in enumerate(sd.query_devices()):
    if TARGET_MIC_NAME in d['name'] and d['max_input_channels'] > 0:
        DEVICE_INDEX = i
        break

if DEVICE_INDEX is None:
    raise RuntimeError(f"❌ マイクが見つかりません: {TARGET_MIC_NAME}")
app.logger.info(f"🎤 使用マイク: {DEVICE_INDEX} - {sd.query_devices()[DEVICE_INDEX]['name']}")

def play_beep_and_record(samplerate, duration):
    app.logger.info("🎙️ ビープと同時に録音開始...")

    try:
        # ビープ音を再生
        try:
            data, fs = sf.read("/app/beep.wav", dtype='float32')
            sd.play(data, fs)
            sd.wait()
        except Exception as e:
            app.logger.warning(f"⚠️ ビープ再生失敗（録音は続行）: {e}")

        # 録音開始
        audio = sd.rec(int(samplerate * duration), samplerate=samplerate, channels=1, device=DEVICE_INDEX)
        sd.wait()

        app.logger.info("✅ 録音完了")
        return audio

    except Exception as e:
        app.logger.error(f"❌ 録音中のエラー: {e}")
        raise


@app.route("/transcribe", methods=["POST"])
def transcribe():
    samplerate = 48000
    duration = 5

    try:
        audio = play_beep_and_record(samplerate, duration)

        audio = np.squeeze(audio).astype(np.float32)
        if np.max(np.abs(audio)) > 0:
            audio = audio / np.max(np.abs(audio))

        sf.write("test.wav", audio, samplerate)
        app.logger.info("💾 WAV保存完了、Whisperで解析中...")

        result = model.transcribe("test.wav", language="en")
        app.logger.info(f"📝 認識結果: {result['text']}")

        return jsonify({"text": result["text"]})

    except Exception as e:
        app.logger.error(f"❌ エラー: {e}")
        return jsonify({"text": "", "error": str(e)}), 500

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5050)