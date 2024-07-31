# Введение в разработку для колесных роботов

Это репозиторий с курсом, который посвящен мобильным роботам. Мы рассмотрим основные компоненты, из которых состоит мобильный робот, какие возникают задачи в процессе создания этих компонент, и какие методы используются для их решения.

Трудоемкость курса рассчитана на 15 часов в неделю.

### Как происходит отбор на курс:
Вам нужно выполнить [отборочное задание](https://github.com/warmhammer/wheeled_robots_intro/tree/main/test_assignment) и отправить свой результат в форму сдачи домашек.

### Список тем:
1. Введение в Linux. Системы котроля версий.
2. Организация проекта на Python
3. Матричные вычисления на Python
4. Базовые алгоритмы робототехники
5. Введение в компьютерное зрение
6. Фреймворк ROS
7. Введение в 3D компьютерное зрение
8. ML для инженеров
9. Оптимизация для инженеров

### Контрольные мероприятия и их сроки
Приблизительно каждую неделю будет выдаваться домашнее задание. Всего планируется около 9 домашних заданий по 10 баллов (включая отборочное задание). Каждое задание выдается на две недели. Штраф за опоздание вычисляется по формуле min(d, 5) * 10%, где d -- количество неполных суток, прошедших после дедлайна. Подробный сроки будут озвучены по ходу курса.

### Критерии оценки
Оценка определяется набранной суммой баллов за домашние задания. Критерии оценивания (в процентах, максимум 100):
* Отлично: 90+
* Хорошо: 75 - 90
* Удовлетворительно: 60 - 75

(нижняя граница включается, верхняя — не включается)

### Правила выполнения ДЗ

Инструкция по сдаче ДЗ живет [здесь](https://github.com/warmhammer/wheeled_robots_intro/tree/main/GIT.md).

Допускается обсуждение домашних заданий с товарищами, но за списывание баллы за работу будут обнуляться у всех сопричастных.

Решения нельзя публиковать в открытом доступе без разрешения преподавателя курса.


## Расписание занятий

| №         | Тема                              | Дата                  | Материалы |
|-----------|---------------------------------------|-----------------------|-----------|
| `1`       | linux, git \| python classes          | 7.07 (Sun), 18:00     | [video](https://youtu.be/gpUaZDVf8Ms), [slides](https://docs.google.com/presentation/d/1bFtvqoGm9mmw2em5YF0iMlQTd0qG3hPaZahPpMy67Y0/edit?usp=sharing) |
| `2`       | venv, pip, docker \| numpy            | 10.07 (Wed), 18:00    | [video](https://youtu.be/h6uJPCfl3W0), [slides](https://docs.google.com/presentation/d/1K1_GDXQQU-d5Oyx6nJNo5DSLzRZGxVDfatIxMsNJnkQ/edit?usp=sharing) |
| `3`       | robotics algorithms \| ekf            | 17.07 (Wed), 18:00    | [video](https://youtu.be/kOqmsgt9zi0), [slides](https://docs.google.com/presentation/d/1d4qIVaG8D9hKw8Xe37uqkCHtaMiPrJuvaDfccJkUkPI/edit?usp=sharing) |
| `4`       | camera calibration \| optimization    | 24.07 (Wed), 18:00    | [video](https://youtu.be/f7-pX7tOpj0), [slides](https://docs.google.com/presentation/d/1kR5cs6kO3cLQ04F0g3_MgHIME2nAgP-ZBN5HD5Li6sY/edit?usp=sharing) |
| `5`       | ros                 | 31.07 (Wed), 18:00    | [video](https://youtu.be/vNQ4BmZ8i2A), [slides](https://docs.google.com/presentation/d/1J6z0YwZLZ6tP3hlSLxahimO6u87USpjpj3sww8npURI/edit?usp=sharing) |
| `6`       |                   | ???  (7.08, Wed)      |           |
| `7`       |                   | 14.08 (Wed), 18:00    |           |
| `8`       |                   | ???                   |           |
| `9`       |                   | ???                   |           |

## Сроки домашних заданий

| №         | Название              | Дедлайн               |
|-----------|-----------------------|-----------------------|
| `hw-0`    | test assignment       | 8.07 (Mon), 23:59     |
| `hw-1`    | pure python           | 21.07 (Sun), 23:59    |
| `hw-2`    | mujoco diff drive     | 28.07 (Sun), 23:59    |
| `hw-3`    | ekf                   | 4.08 (Sun), 23:59     |
| `hw-4`    | camera calib          | 11.08 (Sun), 23:59    |
| `hw-5.1`  | project: lit review   | 11.08 (Sun), 23:59    |
| `hw-5.2`  | project: implement    | 18.08 (Sun), 23:59    |