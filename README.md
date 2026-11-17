# Gorodovoi — Система технического зрения и сопровождения

## Локальный тест на Windows (веб-камера + веб-интерфейс):
1. `cd GorodovoiDesktop/docker/gr5_neural_pipe`
2. `pip install -r requirements.txt`
3. `uvicorn app:app --host 0.0.0.0 --port 8085`
4. Открыть в браузере: http://localhost:8085

## Запуск на роботе (Ubuntu 22.04 + Docker):
1. Скопировать папку `GorodovoiDesktop` на робота
2. `cd ~/GorodovoiDesktop/docker && make up`
