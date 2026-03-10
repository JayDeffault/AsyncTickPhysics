# Vehicle Setup Instructions

## 1) Как настроить `WheelCollisionMesh`

Для каждого `UWheelComponent`:

1. Назначьте `WheelCollisionMesh` (отдельный wheel collision static mesh).
2. Включите параметры:
   - `Hidden In Game = true`
   - `Simulate Physics = false`
   - `Collision Enabled = Query Only`
   - `Collision Response`:
     - `WorldStatic = Block`
     - `WorldDynamic = Block`
     - Остальное = `Ignore`
3. Этот меш используется **только** для sweep-контакта в `UWheelComponent::UpdateContact`.

## 2) Как добавить 4 колеса в конструкторе

В `ABaseVehicle` уже создан пример:

- `Wheel_FrontLeft`
- `Wheel_FrontRight`
- `Wheel_RearLeft`
- `Wheel_RearRight`

Каждое колесо создаётся через `CreateDefaultSubobject<UWheelComponent>(...)`,
аттачится к `BodyMesh` и добавляется в массив `Wheels`.

Рекомендуется:

- Передние колёса: `bIsSteerWheel = true`
- Ведущие колёса задаются через `bIsDrivenWheel` + `UEngineComponent::DriveType`
- Отрегулировать `SetRelativeLocation` под ваш кузов и базу автомобиля.

## 3) Async Tick интеграция

- `ABaseVehicle` наследуется от `AAsyncTickPawn`, но выполняет физическую симуляцию в `NativeAsyncTick`.
- Применение сил к кузову выполняется через `UAsyncTickFunctions` (`ATP_AddForceAtPosition`, `ATP_AddTorque`).
- Дебаг-отрисовка и визуальная часть выполняется в обычном `Tick`.


## 4) Визуальные меши колёс

- У каждого `UWheelComponent` есть `VisualWheelMesh` (отдельный визуальный меш колеса).
- `VisualWheelMesh` не участвует в коллизии (`NoCollision`) и используется только для визуала.
- Во время симуляции колесо:
  - вращается от `WheelAngularVelocity`,
  - поворачивается по рулю для `bIsSteerWheel = true`.
- При отсутствии назначенного меша автоматически подставляется `/Engine/BasicShapes/Cylinder.Cylinder`.
