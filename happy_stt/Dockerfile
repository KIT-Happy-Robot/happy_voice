FROM whisper:r36.4-cu126-22.04-whisper

# モデル先読み：ここを最初に持ってくるとキャッシュが効きやすい
RUN python3 -c "import whisper; whisper.load_model('small')"

# 音声出力と録音に必要なシステムパッケージ
RUN apt-get update && apt-get install -y \
    libasound2 \
    libasound2-dev \
    alsa-utils \
    portaudio19-dev \
    gcc \
    build-essential \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Flask / sounddevice などの追加ライブラリ
RUN pip3 install --no-cache-dir --ignore-installed \
    flask \
    sounddevice \
    soundfile \
    simpleaudio \
    "numpy<2.0.0"

# 作業ディレクトリ
WORKDIR /app

# コード・音声ファイルを配置
COPY app.py /app/app.py
COPY beep.py /app/beep.py
COPY beep.wav /app/beep.wav

# Flask サーバーを起動
CMD ["python3", "/app/app.py"]
