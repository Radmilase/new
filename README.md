# Демоверсия задания полуфинала олимпиады "Я - профессионал" 2023-2024 по робототехнике
[![Telegram](https://img.shields.io/badge/Telegram-2CA5E0?style=for-the-badge&logo=telegram&logoColor=white)](https://t.me/iprofirobots)    [![Yandex IProfi](https://img.shields.io/badge/yandex-%23FF0000.svg?&style=for-the-badge&logo=yandex&logoColor=white)](https://yandex.ru/profi/profile/?page=contests)  [![Mail](https://custom-icon-badges.demolab.com/badge/-iprofi.robotics@yandex.ru-red?style=for-the-badge&logo=mention&logoColor=white)](mailto:iprofi.robotics@yandex.ru)

---
![scene pic](docs/figures/scene_view.png)

---

Репозиторий содержит ROS-пакет с минимальным *решением* задачи. Участнику следует, модифицируя этот пакет, решить задачу.

## Задача

Сцена включает в себя стационарный манипулятор, установленный на столе. Также на столе разбросан _мусор_ в виде алюминиевых банок разных напитков, отличающихся внешним видом.

Вам необходимо с использованием доступных сенсоров реализовать алгоритм управления стационарным манипулятором, который позволит отсортировать банки по двум разным корзинам. В результате сортировки в каждой корзине должны находится банки только одного типа. Не принципиален выбор того, в какую из корзин должны складываться банки определенного типа.


## Как все работает

Доступны два docker-образа:

- `scene-master-demo` - read-only образ, включающий сцену и робота в gazebo. Образ скачивается из регистра gitlab.
- `solution-master-demo-img` - образ с зависимостями для решения задачи. Образ собирается у вас на компьютере. В рамках Демо-версии задания вы можете устанавливать в него любые пакеты, которые вам необходимо аппробироать для того чтобы понять насколько они будут необходимы вам в полуфинале.

Для запуска docker-контейнеров используется инструмент docker-compose. Описание параметров запуска доступно в этом репозитории в файлах:

- `docker-compose.yml ` - если у вас **нет** видеокарты *Nvidia*.
- `docker-compose.nvidia.yml `. - если у вас есть видеокарта от *Nvidia*.


## Установка и настройка окружения
Для настройки окружения необходимо иметь одну из перечисленных операционных систем:
1. Ubuntu 16.04 и старше
2. Windows 10 и старше, с установленным WSL (Не рекомендуется).

Для подготовки окружения необходимо сделать следующее:
1. Установить docker-engine: [Docker Engine](https://docs.docker.com/engine/install/ubuntu/).  
2. Также необходимо установить docker-compose-plugin: [Docker Compose](https://docs.docker.com/compose/install/linux/).  
3. Если вы планируете использовать видеокарту, установите также nviidia-container-toolkit: [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)


## Запуск решения
Склонируйте репозиторий в рабочую директорию:

    git clone https://gitlab.com/beerlab/iprofi2024/demo/master.git
    cd master

Перед запуском на Linux выполните следующую команду:

    xhost +local:docker

Для запуска сцены и этого пакета используйте команду:

    docker compose -f docker-compose.yml up

В случае необходимости пересборки используйте флаг `--build`:

    docker compose -f docker-compose.yml up --build

Для получения последней версии сцены(обновления), используейте флаг `--pull always`:

    docker compose -f docker-compose.yml up --build --pull always


В файле `docker-compose.yml` хранится описание параметров запуска сцены и решения. По умолчанию для решения **автоматически** запускается `start.launch`

    roslaunch solution_master start.launch

Вы можете редактировать все файлы за исключением `docker-compose.yml` и `docker-compose.nvidia.yml`

Для открытия новой bash-сессии используйте команду:

    docker compose exec solution bash

## Быстрый перезапуск решения
Для быстрого изменения и пересборки отредактируйте необходимые файлы и в соседней вкладке перезапустите сервис решения с помощью команды:

    docker compose restart solution



## Оценка

Оценивается колличество синих и красных кубиков, перевезенный в зоны соответствующего цвета за 5 минут. За каждое столкновение с препятствиями назначается штраф: -0.1 балл. За каждый привезенный кубик нужного цвета назначается +1 балл.

