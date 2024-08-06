# Задание 6. ROS
Эта домашка была [заданием полуфинала олимпиады "Я - профессионал" 2023-2024 по робототехнике для бакалавриата](https://gitlab.com/beerlab/iprofi2024/problem/bachelor).

[![Telegram](https://img.shields.io/badge/Telegram-2CA5E0?style=for-the-badge&logo=telegram&logoColor=white)](https://t.me/iprofirobots) [![Yandex IProfi](https://img.shields.io/badge/yandex-%23FF0000.svg?&style=for-the-badge&logo=yandex&logoColor=white)](https://yandex.ru/profi/second_stage) [![Mail](https://custom-icon-badges.demolab.com/badge/-iprofi.robotics@yandex.ru-red?style=for-the-badge&logo=mention&logoColor=white)](mailto:iprofi.robotics@yandex.ru)


![scene pic](docs/figures/scene_view.png)


Репозиторий содержит ROS-пакет с минимальным *решением* задачи. Участнику следует, модифицируя этот пакет, решить задачу.

## Задача

Дано производственное помещение, разбитое на производственные участки, выходы из помещения заблокированы, но можно перемещаться внутри помещения, избегая разделительные ограды между участками производства. Для ликвидации аварии **необходимо собрать радиоактивные материалы в виде синих прямоугольников**, которые разбросаны по площади помещения. Для сбора вредных материалов доступен мобильный колесный робот, сбор осуществляется путем наезда роботом на вредное вещество. Кроме вредных веществ, в помещениях разлиты безвредные вязкие вещества красного цвета, их следует избегать. Также нужно избегать столкновений с объектами окружения (разделительными перегородками).

Участникам предлагается, с использованием доступного робота, реализовать алгоритм управления, который позволит собрать наибольшее количество вредных веществ с минимальным количеством застреваний в безвредных веществах.

В закрытых тестовых сценариях могут быть изменены конфигурация статических препятствий, расположение как вредных, так и безвредных веществ.

Требуется разработать техническое решение, включая алгоритмическое и программное обеспечения системы управления и обработки сенсорной информации, в форме программного пакета для ROS на языках программирования С++ и/или Python.


## Как все работает

Для решения задачи доступны два read-only docker-образа:

- [base] `registry.gitlab.com/beerlab/iprofi2024/problem/bachelor/base-user:latest` -- включает все зависимости.

- [scene] `registry.gitlab.com/beerlab/iprofi2024/problem/bachelor/scene:latest` -- собран на базе предыдущего и дополнительно включает файлы сцены в gazebo.

Запуск включает два шага:
- В контейнере сервиса `scene` на основе образа `[scene]` запускается сцена в симуляторе gazebo [scene_bachelor](https://gitlab.com/beerlab/iprofi2024_dev/problem/bachelor_scene).
- В контейнере сервиса `solution` на основе образа `[base]` запускается решение [solution_bachelor](https://gitlab.com/beerlab/iprofi2024/problem/bachelor).

Для автоматизации запуска запуска docker-контейнеров используется инструмент docker compose. Описание параметров запуска доступно в: `docker-compose.yml` и `docker-compose.nvidia.yml`.

*Note! Если вы используется систему с GPU от Nvidia используйте `docker-compose.nvidia.yml`*


## Установка и настройка окружения

Для настройки окружения необходимо иметь одну из перечисленных операционных систем:
1. Ubuntu 16.04 и старше
2. Windows 10 и старше, с установленным WSL2 (Не рекомендуется).

Для подготовки окружения необходимо сделать следующее:
1. Установить docker-engine: [Docker Engine](https://docs.docker.com/engine/install/ubuntu/).  
2. Также необходимо установить docker-compose-plugin: [Docker Compose](https://docs.docker.com/compose/install/linux/).  
3. Если вы планируете использовать видеокарту, установите также nviidia-container-toolkit: [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
4. Добавить в группу docker пользователя

    ```bash
    sudo groupadd docker 
    sudo usermod -aG docker $USER 
    newgrp docker
    ```

## Как запустить начальное(базовое решение)
**Сделать форк репозитория**([как сделать форк](https://docs.gitlab.com/ee/user/project/repository/forking_workflow.html)) или **импорт**([как сделать импорт](https://docs.github.com/en/migrations/importing-source-code/using-github-importer/importing-a-repository-with-github-importer)) в случае использования Github.  
Установить параметр видмости: **Private**.  

Склонировать репозиторий:

```bash
git clone <ССЫЛКА НА ФОРК ИЛИ ИМПОРТ РЕПОЗИТОРИЙ РЕШЕНИЯ >
cd bachelor
```

Дать права для подключения группе docker к дисплею хоста:

```
xhost +local:docker
```

Запустить сцену и ros-пакет из этого репозитория:

```bash
docker compose -f docker-compose.yml up --build --pull always
```
*Note!* В файле `docker-compose.yml` и `docker-compose.nvidia.yml` хранится описание параметров запуска сцены и решения.

### Редактирование базового решения
Для редактирования доступны все файлы в репозтории, за исключение файлов `docker-compose*.yml`.  
Чтобы начать решать задание вы можете отредактировать файл `start.launch` выбрав запуск python или C++ версии программы решения. 

Если вы пишете на python, нужно, чтобы в `start.launch` была раскомментирована строка: 

    <node name="example_node" pkg="solution_bachelor" type="example.py" output="screen"></node>

Если вы пишете на C++, нужно, чтобы в `start.launch` была раскомментирована строка: 

    <node name="example_node" pkg="solution_bachelor" type="example_node" output="screen"></node>

## Дополнительные полезные команды

В случае необходимости пересборки используйте флаг `--build`:

    docker compose -f docker-compose.yml up --build

Для получения последней версии сцены (обновления) используейте флаг `--pull always`:

    docker compose -f docker-compose.yml up --build --pull always

### Подключение в контейнер

Для открытия новой bash-сессии в сервисе решения: `solution` используйте команду:

    docker compose exec solution bash

Для открытия новой bash-сессии в сервисе сцены: `scene` используйте команду:

    docker compose exec scene bash

### Рестарт сцены или решения по отдельности
Для перезапуска **решения** используйте:

    docker compose restart solution

Для перезапуска **сцены** используйте:

    docker compose restart scene

###  Включение/выключение Rviz

При необходимости вы можете отключить Rviz, для этого нужно отредактировать файл `docker-compose.yml` или `docker-compose.nvidia.yml`, заменив `RVIZ_GUI=true` на `RVIZ_GUI=false`.

## Отправка на тестирование
При отправке на тестирование убедитесь, что ваше решение не создает графических окон при установлении параметра `GUI=false` в `docker-compose.yml` или `docker-compose.nvidia.yml`, для этого при разработке программы, вы можете использовать переменную окружения `GUI` с помощью: 
```c++
std::string gui = std::getenv("GUI");
```
или для _Python3_:
```python3
gui = os.getenv('GUI')
```

## Оценка

Оценивается количество собранных роботом элементов вредного и безвредного веществ.

- 1 балл – за каждый собранный элемент вредного вещества типа 1 (синий)
- 2 балла – за каждый собранный элемент вредного вещества типа 2 (синий)
- 1 балл штрафа – за каждый наезд на безвредные вещества (красный)

Время на выполнение задания роботом в симуляторе **10 минут**.


Максимальный балл, полученный в прогоне будет считаться вашей оценкой. За домашку можно получить до 20 баллов (10 из них считаются бонусными). 

Не будут приниматься "закардкоженные" решения и решения, которые ломают симулятор или рефери. Вы так же можете приложить ссылку на видеозапись с прогоном для упрощения проверки (будут смотреться только первые 10 минут времени симуляции).