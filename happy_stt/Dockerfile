FROM python:3.10-slim

# システムパッケージ（ALSA/音声再生/C拡張用）
RUN apt-get update && apt-get install -y \
    gcc \
    build-essential \
    libasound2 \
    libasound2-dev \
    alsa-utils \
    portaudio19-dev \
    ffmpeg \ 
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Python依存ライブラリ（Whisper含む）
RUN pip install --no-cache-dir --ignore-installed \
    flask \
    sounddevice \
    soundfile \
    simpleaudio \
    numpy \
    torch \
    openai-whisper

# Whisperモデル（small）を事前ダウンロード
RUN python3 -c "import whisper; whisper.load_model('small')"

# アプリ用ファイル配置
WORKDIR /app
COPY app.py /app/app.py
COPY beep.py /app/beep.py
COPY beep.wav /app/beep.wav

CMD ["python3", "/app/app.py"]
