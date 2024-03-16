#!/bin/bash

if [[ "$1" == "llava" ]]; then
    cd LLaVA && llava_venv/bin/python server.py --model_name "$2" --load_in_4bit
elif [[ "$1" == "honeybee" ]]; then
    cd honeybee && honeybee_venv/bin/python server.py --model_name "$2" --use_bf16
else
    echo "Invalid argument. Please specify 'llava' or 'honeybee'."
fi
