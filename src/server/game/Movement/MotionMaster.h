/*
 * This file is part of the TrinityCore Project. See AUTHORS file for Copyright information
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MOTIONMASTER_H
#define MOTIONMASTER_H

#include "Common.h"
#include "Duration.h"
#include "ObjectGuid.h"
#include "Optional.h"
#include "MovementDefines.h"
#include "MovementGenerator.h"
#include "SharedDefines.h"
#include <deque>
#include <functional>
#include <set>
#include <unordered_map>
#include <vector>

class PathGenerator;
class Unit;
struct Position;
struct SplineChainLink;
struct SplineChainResumeInfo;
struct WaypointPath;
enum UnitMoveType : uint8;

namespace Movement
{
    class MoveSplineInit;
    struct SpellEffectExtraData;
}

enum MotionMasterFlags : uint8
{
    MOTIONMASTER_FLAG_NONE = 0x0,
    MOTIONMASTER_FLAG_UPDATE = 0x1,                        // Update in progress
    MOTIONMASTER_FLAG_STATIC_INITIALIZATION_PENDING = 0x2, // Static movement (MOTION_SLOT_DEFAULT) hasn't been initialized
    MOTIONMASTER_FLAG_INITIALIZATION_PENDING = 0x4,        // MotionMaster is stalled until signaled
    MOTIONMASTER_FLAG_INITIALIZING = 0x8,                  // MotionMaster is initializing

    MOTIONMASTER_FLAG_DELAYED = MOTIONMASTER_FLAG_UPDATE | MOTIONMASTER_FLAG_INITIALIZATION_PENDING
};

enum MotionMasterDelayedActionType : uint8
{
    MOTIONMASTER_DELAYED_CLEAR = 0,
    MOTIONMASTER_DELAYED_CLEAR_SLOT,
    MOTIONMASTER_DELAYED_CLEAR_MODE,
    MOTIONMASTER_DELAYED_CLEAR_PRIORITY,
    MOTIONMASTER_DELAYED_ADD,
    MOTIONMASTER_DELAYED_REMOVE,
    MOTIONMASTER_DELAYED_REMOVE_TYPE,
    MOTIONMASTER_DELAYED_INITIALIZE
};

struct MovementGeneratorDeleter
{
    void operator()(MovementGenerator *a);
};

struct MovementGeneratorComparator
{
public:
    bool operator()(MovementGenerator const *a, MovementGenerator const *b) const;
};

struct MovementGeneratorInformation
{
    MovementGeneratorInformation(MovementGeneratorType type, ObjectGuid targetGUID, std::string const &targetName);

    MovementGeneratorType Type;
    ObjectGuid TargetGUID;
    std::string TargetName;
};

static bool EmptyValidator()
{
    return true;
}

class TC_GAME_API MotionMaster
{
public:
    typedef std::function<void()> DelayedActionDefine;
    typedef std::function<bool()> DelayedActionValidator;

    class DelayedAction
    {
    public:
        explicit DelayedAction(DelayedActionDefine &&action, DelayedActionValidator &&validator, MotionMasterDelayedActionType type) : Action(std::move(action)), Validator(std::move(validator)), Type(type) {}
        explicit DelayedAction(DelayedActionDefine &&action, MotionMasterDelayedActionType type) : Action(std::move(action)), Validator(EmptyValidator), Type(type) {}
        ~DelayedAction() {}

        void Resolve()
        {
            if (Validator())
                Action();
        }

        DelayedActionDefine Action;
        DelayedActionValidator Validator;
        uint8 Type;
    };

    explicit MotionMaster(Unit *unit);
    ~MotionMaster();

    void Initialize();
    void InitializeDefault();
    void AddToWorld();

    bool Empty() const;
    uint32 Size() const;
    std::vector<MovementGeneratorInformation> GetMovementGeneratorsInformation() const;
    MovementSlot GetCurrentSlot() const;
    MovementGenerator *GetCurrentMovementGenerator() const;
    MovementGeneratorType GetCurrentMovementGeneratorType() const;
    MovementGeneratorType GetCurrentMovementGeneratorType(MovementSlot slot) const;
    MovementGenerator *GetCurrentMovementGenerator(MovementSlot slot) const;
    // Returns first found MovementGenerator that matches the given criteria
    MovementGenerator *GetMovementGenerator(std::function<bool(MovementGenerator const *)> const &filter, MovementSlot slot = MOTION_SLOT_ACTIVE) const;
    bool HasMovementGenerator(std::function<bool(MovementGenerator const *)> const &filter, MovementSlot slot = MOTION_SLOT_ACTIVE) const;

    void Update(uint32 diff);
    void Add(MovementGenerator *movement, MovementSlot slot = MOTION_SLOT_ACTIVE);
    // NOTE: MOTION_SLOT_DEFAULT will be autofilled with IDLE_MOTION_TYPE
    void Remove(MovementGenerator *movement, MovementSlot slot = MOTION_SLOT_ACTIVE);
    // Removes first found movement
    // NOTE: MOTION_SLOT_DEFAULT will be autofilled with IDLE_MOTION_TYPE
    void Remove(MovementGeneratorType type, MovementSlot slot = MOTION_SLOT_ACTIVE);
    // NOTE: MOTION_SLOT_DEFAULT wont be affected
    void Clear();
    // Removes all movements for the given MovementSlot
    // NOTE: MOTION_SLOT_DEFAULT will be autofilled with IDLE_MOTION_TYPE
    void Clear(MovementSlot slot);
    // Removes all movements with the given MovementGeneratorMode
    // NOTE: MOTION_SLOT_DEFAULT wont be affected
    void Clear(MovementGeneratorMode mode);
    // Removes all movements with the given MovementGeneratorPriority
    // NOTE: MOTION_SLOT_DEFAULT wont be affected
    void Clear(MovementGeneratorPriority priority);
    void PropagateSpeedChange();
    bool GetDestination(float &x, float &y, float &z);
    bool StopOnDeath();
    // idle动画什么的
    void MoveIdle();
    // 朝预先设计的点
    void MoveTargetedHome();
    // 随机移动
    void MoveRandom(float wanderDistance = 0.0f, Optional<Milliseconds> duration = {}, MovementSlot slot = MOTION_SLOT_DEFAULT,
                    Optional<Scripting::v2::ActionResultSetter<MovementStopReason>> &&scriptResult = {});
    // 跟随移动
    void MoveFollow(Unit *target, float dist, Optional<ChaseAngle> angle = {}, Optional<Milliseconds> duration = {}, bool ignoreTargetWalk = false, MovementSlot slot = MOTION_SLOT_ACTIVE,
                    Optional<Scripting::v2::ActionResultSetter<MovementStopReason>> &&scriptResult = {});
    // 追逐
    void MoveChase(Unit *target, Optional<ChaseRange> dist = {}, Optional<ChaseAngle> angle = {});
    void MoveChase(Unit *target, float dist, float angle) { MoveChase(target, ChaseRange(dist), ChaseAngle(angle)); }
    // 困惑 如致盲？？
    void MoveConfused();
    // 逃跑
    void MoveFleeing(Unit *enemy, Milliseconds time = 0ms,
                     Optional<Scripting::v2::ActionResultSetter<MovementStopReason>> &&scriptResult = {});
    // 朝指定坐标移动
    void MovePoint(uint32 id, Position const &pos, bool generatePath = true, Optional<float> finalOrient = {}, Optional<float> speed = {},
                   MovementWalkRunSpeedSelectionMode speedSelectionMode = MovementWalkRunSpeedSelectionMode::Default, Optional<float> closeEnoughDistance = {},
                   Optional<Scripting::v2::ActionResultSetter<MovementStopReason>> &&scriptResult = {});
    void MovePoint(uint32 id, float x, float y, float z, bool generatePath = true, Optional<float> finalOrient = {}, Optional<float> speed = {},
                   MovementWalkRunSpeedSelectionMode speedSelectionMode = MovementWalkRunSpeedSelectionMode::Default, Optional<float> closeEnoughDistance = {},
                   Optional<Scripting::v2::ActionResultSetter<MovementStopReason>> &&scriptResult = {});
    /*
     *  Makes the unit move toward the target until it is at a certain distance from it. The unit then stops.
     *  Only works in 2D.
     *  This method doesn't account for any movement done by the target. in other words, it only works if the target is stationary.
     *  使单位向目标移动，直到它与目标保持一定距离。然后机器停止工作。
     *  只适用于2D。
     *  这个方法不考虑目标的任何移动。换句话说，它只在目标静止时起作用。
     */
    void MoveCloserAndStop(uint32 id, Unit *target, float distance);
    // These two movement types should only be used with creatures having landing/takeoff animations
    // 着陆
    void MoveLand(uint32 id, Position const &pos, Optional<int32> tierTransitionId = {}, Optional<float> velocity = {},
                  MovementWalkRunSpeedSelectionMode speedSelectionMode = MovementWalkRunSpeedSelectionMode::Default,
                  Optional<Scripting::v2::ActionResultSetter<MovementStopReason>> &&scriptResult = {});
    // 起飞？
    void MoveTakeoff(uint32 id, Position const &pos, Optional<int32> tierTransitionId = {}, Optional<float> velocity = {},
                     MovementWalkRunSpeedSelectionMode speedSelectionMode = MovementWalkRunSpeedSelectionMode::Default,
                     Optional<Scripting::v2::ActionResultSetter<MovementStopReason>> &&scriptResult = {});
    void MoveCharge(float x, float y, float z, float speed = SPEED_CHARGE, uint32 id = EVENT_CHARGE, bool generatePath = false, Unit const *target = nullptr, Movement::SpellEffectExtraData const *spellEffectExtraData = nullptr);
    void MoveCharge(PathGenerator const &path, float speed = SPEED_CHARGE, Unit const *target = nullptr, Movement::SpellEffectExtraData const *spellEffectExtraData = nullptr);
    // 击退
    void MoveKnockbackFrom(Position const &origin, float speedXY, float speedZ, Movement::SpellEffectExtraData const *spellEffectExtraData = nullptr);
    void MoveJumpTo(float angle, float speedXY, float speedZ);
    void MoveJump(Position const &pos, float speedXY, float speedZ, uint32 id = EVENT_JUMP, MovementFacingTarget const &facing = {},
                  bool orientationFixed = false, JumpArrivalCastArgs const *arrivalCast = nullptr, Movement::SpellEffectExtraData const *spellEffectExtraData = nullptr,
                  Optional<Scripting::v2::ActionResultSetter<MovementStopReason>> &&scriptResult = {});
    void MoveJump(float x, float y, float z, float speedXY, float speedZ, uint32 id = EVENT_JUMP, MovementFacingTarget const &facing = {},
                  bool orientationFixed = false, JumpArrivalCastArgs const *arrivalCast = nullptr, Movement::SpellEffectExtraData const *spellEffectExtraData = nullptr,
                  Optional<Scripting::v2::ActionResultSetter<MovementStopReason>> &&scriptResult = {});
    void MoveJumpWithGravity(Position const &pos, float speedXY, float gravity, uint32 id = EVENT_JUMP, MovementFacingTarget const &facing = {},
                             bool orientationFixed = false, JumpArrivalCastArgs const *arrivalCast = nullptr, Movement::SpellEffectExtraData const *spellEffectExtraData = nullptr,
                             Optional<Scripting::v2::ActionResultSetter<MovementStopReason>> &&scriptResult = {});
    // 原型路径移动
    void MoveCirclePath(float x, float y, float z, float radius, bool clockwise, uint8 stepCount,
                        Optional<Milliseconds> duration = {}, Optional<float> speed = {},
                        MovementWalkRunSpeedSelectionMode speedSelectionMode = MovementWalkRunSpeedSelectionMode::Default,
                        Optional<Scripting::v2::ActionResultSetter<MovementStopReason>> &&scriptResult = {});
    // Walk along spline chain stored in DB (script_spline_chain_meta and script_spline_chain_waypoints)
    // 遍历存储在DB中的样条链（script_spline_chain_meta和script_spline_chain_waypoints）
    void MoveAlongSplineChain(uint32 pointId, uint16 dbChainId, bool walk);
    void MoveAlongSplineChain(uint32 pointId, std::vector<SplineChainLink> const &chain, bool walk);
    void ResumeSplineChain(SplineChainResumeInfo const &info);
    void MoveFall(uint32 id = 0,
                  Optional<Scripting::v2::ActionResultSetter<MovementStopReason>> &&scriptResult = {});
    void MoveSeekAssistance(float x, float y, float z);
    void MoveSeekAssistanceDistract(uint32 timer);
    void MoveTaxiFlight(uint32 path, uint32 pathnode, Optional<float> speed = {},
                        Optional<Scripting::v2::ActionResultSetter<MovementStopReason>> &&scriptResult = {});
    void MoveDistract(uint32 time, float orientation);
    void MovePath(uint32 pathId, bool repeatable, Optional<Milliseconds> duration = {}, Optional<float> speed = {},
                  MovementWalkRunSpeedSelectionMode speedSelectionMode = MovementWalkRunSpeedSelectionMode::Default,
                  Optional<std::pair<Milliseconds, Milliseconds>> waitTimeRangeAtPathEnd = {}, Optional<float> wanderDistanceAtPathEnds = {},
                  Optional<bool> followPathBackwardsFromEndToStart = {}, Optional<bool> exactSplinePath = {}, bool generatePath = true,
                  Optional<Scripting::v2::ActionResultSetter<MovementStopReason>> &&scriptResult = {});
    void MovePath(WaypointPath const &path, bool repeatable, Optional<Milliseconds> duration = {}, Optional<float> speed = {},
                  MovementWalkRunSpeedSelectionMode speedSelectionMode = MovementWalkRunSpeedSelectionMode::Default,
                  Optional<std::pair<Milliseconds, Milliseconds>> waitTimeRangeAtPathEnd = {}, Optional<float> wanderDistanceAtPathEnds = {},
                  Optional<bool> followPathBackwardsFromEndToStart = {}, Optional<bool> exactSplinePath = {}, bool generatePath = true,
                  Optional<Scripting::v2::ActionResultSetter<MovementStopReason>> &&scriptResult = {});

    /**
     * \brief Makes the Unit turn in place
     * \param id Movement identifier, later passed to script MovementInform hooks
     * \param direction Rotation direction
     * \param time How long should this movement last, infinite if not set
     * \param turnSpeed How fast should the unit rotate, in radians per second. Uses unit's turn speed if not set
     * \param totalTurnAngle Total angle of the entire movement, infinite if not set
     * \param scriptResult Awaitable script result (for internal use)
     * \brief 使单元旋转到位
     * \param id移动标识符，稍后传递给脚本MovementInform钩子
     * \param direction旋转方向
     * \param  这个运动应该持续多长时间，如果没有设置则为无限
     * \param turnSpeed单位旋转的速度，单位为弧度每秒。使用单位的旋转速度，如果没有设置
     * \param totalTurnAngle整个移动的总角度，如果没有设置则无限
     * \param scriptResult可等待的脚本结果（内部使用）
     */

    void MoveRotate(uint32 id, RotateDirection direction, Optional<Milliseconds> time = {},
                    Optional<float> turnSpeed = {}, Optional<float> totalTurnAngle = {},
                    Optional<Scripting::v2::ActionResultSetter<MovementStopReason>> &&scriptResult = {});
    void MoveFormation(Unit *leader, float range, float angle, uint32 point1, uint32 point2);

    void LaunchMoveSpline(std::function<void(Movement::MoveSplineInit &init)> &&initializer, uint32 id = 0, MovementGeneratorPriority priority = MOTION_PRIORITY_NORMAL, MovementGeneratorType type = EFFECT_MOTION_TYPE);
    /*
        该函数的主要任务是计算跳跃相关的速度参数，
        用于确定在游戏中单位进行跳跃动作时在不同方向上的速度，以便实现符合物理规律或者游戏设计要求的跳跃效果，
        比如角色跨越沟壑、爬上高台等跳跃场景下需要合理的速度来保证跳跃的距离、高度等符合预期。
    */
    void CalculateJumpSpeeds(float dist, UnitMoveType moveType, float speedMultiplier, float minHeight, float maxHeight, float &speedXY, float &speedZ) const;

private:
    typedef std::unique_ptr<MovementGenerator, MovementGeneratorDeleter> MovementGeneratorPointer;
    typedef std::multiset<MovementGenerator *, MovementGeneratorComparator> MotionMasterContainer;
    typedef std::unordered_multimap<uint32, MovementGenerator const *> MotionMasterUnitStatesContainer;

    void AddFlag(uint8 const flag) { _flags |= flag; }
    bool HasFlag(uint8 const flag) const { return (_flags & flag) != 0; }
    void RemoveFlag(uint8 const flag) { _flags &= ~flag; }

    void ResolveDelayedActions();
    void Remove(MotionMasterContainer::iterator iterator, bool active, bool movementInform);
    void Pop(bool active, bool movementInform);
    void DirectInitialize();
    void DirectClear();
    void DirectClearDefault();
    void DirectClear(std::function<bool(MovementGenerator *)> const &filter);
    void DirectAdd(MovementGenerator *movement, MovementSlot slot);

    void Delete(MovementGenerator *movement, bool active, bool movementInform);
    void DeleteDefault(bool active, bool movementInform);
    void AddBaseUnitState(MovementGenerator const *movement);
    void ClearBaseUnitState(MovementGenerator const *movement);
    void ClearBaseUnitStates();

    Unit *_owner;
    MovementGeneratorPointer _defaultGenerator;
    MotionMasterContainer _generators;
    MotionMasterUnitStatesContainer _baseUnitStatesMap;
    std::deque<DelayedAction> _delayedActions;
    uint8 _flags;
};

#endif // MOTIONMASTER_H
