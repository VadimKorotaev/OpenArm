# OpenArm Skill Studio

Конструктор навыков (Skills) для bimanual-робота **OpenArm v2.0** в конфигурации
`default_bimanual`.

Оператор через веб-интерфейс собирает последовательность действий — движения
рук по целевой pose end effector, команды захвата, паузы, одновременные блоки —
сохраняет её как Skill и запускает повторно. Для основного сценария положения
рук хранятся **относительно ArUco-маркера**, поэтому при перемещении маркера тот
же Skill без редактирования отрабатывает в новом месте. Мировая система
координат также доступна как вспомогательный режим для отладки.

```
Web UI ──HTTP──> FastAPI ──rclpy──> MoveIt 2 ──> ros2_control (mock) ──> RViz
                    │
                    └── SQLite (Skills, маркеры)
```

---


---

## Быстрый старт

### Что нужно

- Ubuntu 22.04, **ROS 2 Humble** (`/opt/ros/humble`)
- **Python 3.10+**
- `colcon`, `git`, MoveIt 2 (`ros-humble-moveit`), `ros-humble-ros2-control`,
  `ros-humble-ros2-controllers`, `python3-pytest`

### 1. Исходники ROS и сборка

Пакеты OpenArm подключаются как внешние — в репозитории их нет (меши
`openarm_description` весят около 150 МБ).

```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/enactic/openarm_description.git    # проверено с package version 1.0.0
git clone https://github.com/enactic/openarm_ros2.git           # проверено с package version 0.3.0
cd ../..
./scripts/patch_upstream_moveit.sh
cd ros2_ws
colcon build --symlink-install --parallel-workers 4 \
  --packages-select openarm_description openarm_bimanual_moveit_config openarm_bringup
cd ..
```

### 2. Python-окружение

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt
```

Флаг `--system-site-packages` нужен, чтобы Python-процесс видел установленные с
ROS 2 модули `rclpy`, `tf2_ros`, сообщения и action-типы.

### 3. Запуск

Терминал 1 — симуляция (MoveIt + mock hardware + RViz):

```bash
./scripts/start_mock_bimanual.sh
```

Терминал 2 — API и веб-интерфейс:

```bash
./scripts/start_api.sh
```

Открыть **http://127.0.0.1:8000**. Проверка здоровья стека — `curl
localhost:8000/health` или `./scripts/check_mock_bimanual.sh`.

---

## API

`http://127.0.0.1:8000`, интерактивная схема — `/docs`.

**Skills**

| Метод | Путь | |
|---|---|---|
| `GET` | `/api/skills` | список (`limit`, `offset`) |
| `POST` | `/api/skills` | создать |
| `GET` | `/api/skills/{id}` | получить |
| `PATCH` | `/api/skills/{id}` | изменить |
| `DELETE` | `/api/skills/{id}` | удалить |
| `POST` | `/api/skills/{id}/actions/capture-pose?arm=left` | дописать текущую позу как `move` |

**Исполнение**

| Метод | Путь | |
|---|---|---|
| `POST` | `/api/skills/{id}/executions` | запустить |
| `GET` | `/api/executions` · `/api/executions/{id}` | статус и текущее действие |
| `POST` | `/api/executions/{id}/cancel` | отменить |

**Робот и маркеры**

| Метод | Путь | |
|---|---|---|
| `GET` | `/api/robot/arms/{arm}/pose` | текущая поза (`reference_kind`, `frame_id`) |
| `POST` | `/api/robot/arms/{arm}/nudge` | шаг позиции XYZ или поворота RPY в мировых осях |
| `GET` `PUT` `DELETE` | `/api/markers[/{frame_id}]` | TF маркеров |
| `GET` | `/health` | состояние БД и ROS |

В HTTP API поле `rotation_rpy` команды `nudge` передаётся в радианах. Web UI
показывает градусы и выполняет перевод перед запросом.

---

## Ручная проверка

Предзагруженных Skills нет: все сценарии создаются оператором через Web UI и
сохраняются в SQLite. Ручное позиционирование позволяет выбрать руку, двигать
end effector кнопками `±X/±Y/±Z` с шагом 5–20 мм и поворачивать кнопками
`±Roll/±Pitch/±Yaw` с шагом 1–15°. После позиционирования кнопка «Сохранить
текущую pose в Skill» добавляет действие `move` в выбранной системе координат.

Минимальный **single-arm** сценарий для демонстрации:

1. Опубликовать `aruco_marker_1` и выбрать `marker` при сохранении pose.
2. Собрать `move → move → gripper(close) → wait → move → gripper(open)` для
   одной руки.
3. Сохранить Skill, запустить и наблюдать результат в RViz и панели исполнения.

Минимальный **bimanual** сценарий:

1. Добавить последовательное действие, например `wait` или начальный `move`.
2. Добавить `parallel` с отдельной ветвью для левой и правой руки.
3. В каждой ветви задать marker-relative `move`; при необходимости добавить
   команды захватов.
4. Сохранить и запустить Skill. Одновременное движение выполняют разные
   `JointTrajectoryController` рук.

Проверка marker-relative сценария:

```bash
# 1. запуск
curl -X POST localhost:8000/api/skills/<id>/executions

# 2. переместить маркер на 40 мм по Y, не меняя высоту
curl -X PUT localhost:8000/api/markers/aruco_marker_1 \
  -H 'Content-Type: application/json' \
  -d '{"position":{"x":0.30,"y":-0.04,"z":0.50},"orientation":{"w":1.0}}'

# 3. запустить ТОТ ЖЕ Skill без редактирования
curl -X POST localhost:8000/api/skills/<id>/executions

# 4. убедиться, что рука пришла в новую точку
curl "localhost:8000/api/robot/arms/left/pose"
```

Маркер и оба end effector видны в RViz как оси с подписями — остальные фреймы
робота в конфиге выключены, чтобы сцена читалась.

---

## Тесты

```bash
./scripts/run_model_tests.sh
```

31 тест: доменная модель и её валидация, трансформации маркера,
последовательное и параллельное исполнение с отменой, конфликт рук в `parallel`,
семплирование и выравнивание траекторий по общему времени, персистентность
Skills и маркеров через перезапуск приложения, CRUD и отдача статики. Отдельно
проверяются зеркальное преобразование `opening` для двух захватов и наличие
RPY-контролов Web UI.

Отдельно, при поднятом стеке — проверка pose-based IK на живом MoveIt:

```bash
./scripts/verify_both_arms_ik.sh
```

## Структура

```
backend/openarm_skills/   доменная модель, репозиторий, исполнение, ROS-адаптер, API
backend/static/           веб-интерфейс (vanilla JS, без сборки)
backend/tests/            pytest
config/openarm_skills.rviz  RViz-конфиг проекта (TF маркера, обзорная камера)
scripts/                  запуск стека, API, тесты и патчи
data/openarm_skills.db    SQLite (создаётся автоматически)
ros2_ws/                  colcon-воркспейс с внешними пакетами OpenArm
```
