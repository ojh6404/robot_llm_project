#!/usr/bin/env sh

# virtualenv for honeybee
honeybee_setup() {
    git clone https://github.com/kakaobrain/honeybee.git /root/honeybee &&\
        cd /root/honeybee &&\
        python -m venv honeybee_venv &&\
        honeybee_venv/bin/pip install --upgrade pip setuptools wheel &&\
        if [ "$CUDA_VERSION" = "11.8.0" ]; then\
            honeybee_venv/bin/pip install torch==2.1.1 torchvision==0.16.1 torchaudio==2.1.1 --index-url https://download.pytorch.org/whl/cu118;\
        elif [ "$CUDA_VERSION" = "12.1.0" ]; then\
            honeybee_venv/bin/pip install torch==2.1.1 torchvision==0.16.1 torchaudio==2.1.1 --index-url https://download.pytorch.org/whl/cu121;\
        else\
            echo "CUDA_VERSION $CUDA_VERSION is not supported";
            exit 1;\
        fi &&\
        honeybee_venv/bin/pip install -r requirements.txt &&\
        honeybee_venv/bin/pip install -r requirements_demo.txt &&\
        honeybee_venv/bin/pip install flask sentencepiece opencv-python
}

# virtualenv for llava
llava_setup() {
    git clone https://github.com/haotian-liu/LLaVA.git /root/LLaVA &&\
        cd /root/LLaVA &&\
        python -m venv llava_venv &&\
        llava_venv/bin/pip install --upgrade pip setuptools wheel &&\
        llava_venv/bin/pip install -e . &&\
        llava_venv/bin/pip install flask opencv-python protobuf
}
