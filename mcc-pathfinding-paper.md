# 基于 A* 的 Minecraft 机器人寻路系统改进：路径平滑、3D 邻域建模与安全执行

**作者**：K2CrO4__（与 Codex 协作开发）
**日期**：2026-08-02
**代码库**：`MCCTeam/Minecraft-Console-Client`（MCC，C#/.NET 10，Minecraft 26.2 实测）
**基线版本**：`7b100a31`（克隆时 HEAD）

---

## 摘要

针对 Minecraft Console Client（MCC）内置 A* 寻路在复杂地形中暴露的系列缺陷——路径逐格折线导致贴边行走、无法规划 1 格高台阶的斜上过渡导致爬楼梯失败、跳跃中途提前判定"到达"导致从无护栏楼梯滑落、1 格宽平台上踏空坠落、卡拐角与卡墙后的无限重规划、爬梯子被方块碰撞卡死、远目标一次搜索数百万节点——本文提出**两代共 19 项改进**，覆盖**路径规划层**与**运动执行层**两个层面。第一代（第 3.1–3.9 节）解决基础地形可通行性问题：路径平滑（视线裁剪）、3D 邻域建模（斜上跨步与完整方块列检查）、斜向移动安全约束、物理一致的到达判定、卡住检测与自动重规划、原版风格自动跳跃、边缘探测与 1 格宽路径居中校正。第二代（第 3.10–3.19 节）针对大型迷宫与实机联机场景：JPS 跳点搜索、双向 A*、真实距离启发式、异步搜索、`-f` 风险模式下的虚空拦截与"服务器地形信任"、路径执行朝向接管、墙碰撞定点解卡、**爬梯执行层（梯子非碰撞、居中爬升、顶翻越跳跃）**、联机位置同步与未知地面、搜索预算与重规划计数修复、**路上规划（节点预算部分路径 + 自动分段续段）**与实体环境适配（服务器回弹检测、区块流式等待）。全部改进经 48/48 单元测试与 Minecraft 26.2 本地服务器多轮实机验证：机器人先穿越"9 级无护栏楼梯 + 楼顶 + 1 格宽平台 + 深坑"障碍路线；随后在 Backrooms 迷宫完成 2 分 00 秒/160 格长距离导航，并**完整爬过多架 1 格宽梯子**（y=19→23 等）翻上高层平台；240+ 格远目标通过分段规划在约 2 分钟首段等待后开始行进并持续推进到 -331。

**关键词**：Minecraft；A* / JPS 寻路；双向搜索；异步路径搜索；机器人运动规划；路径平滑；自动跳跃；爬梯；MCC

---

## 1 引言

MCC 是跨平台的 Minecraft Java 版文字终端客户端，其自动化能力依赖内置的 A* 方块网格寻路与自研的 1.21.11 物理引擎。在平坦开阔地形上原有寻路可用，但在人工建筑（楼梯、楼顶、1 格宽平台、深坑）构成的复杂场景中，实测出现以下可复现问题：

| 编号 | 现象 | 根因（分析） |
| --- | --- | --- |
| P1 | 机器人贴边、锯齿状行走 | 路径为逐格中心折线，无直线化 |
| P2 | 爬不上 1 格高楼梯，原地跳 | 移动生成器缺少"斜上跨步"过渡，仅能同列直跳（目标空中无支撑） |
| P3 | 爬楼梯后从边缘滑落 | 跳跃中途即判定"到达"并推进下一路点 |
| P4 | 站在 1 格宽平台上踏空坠落 | 执行层不探测前方脚下是否有地面 |
| P5 | 卡在拐角/墙边反复跳 | 无卡住检测与重规划，且重规划可无限循环 |
| P6 | 楼顶直线路径穿过缺口坠落 | 路径平滑未检查采样列的脚下支撑 |
| P7 | 1 格宽平台斜穿拐角扫到深坑 | 斜向移动未检查两侧直角格安全性 |
| P8 | 大型迷宫搜索 40 万+ 节点超时 | 平方距离启发式严重高估 H，A* 退化为贪心 |
| P9 | `-f` 模式走进无底虚空被服务器弹回 | 无支撑虚空格被当作可行走，执行层又误信陈旧区块缓存 |
| P10 | 原地"小碎步"永不前进 | 服务器位置包把朝向钉回旧值抵消移动方向；碰撞解卡差 0.07 格；重规划计数被平方距离杠杆效应卡死 |

本文第 2 节简述系统现状；第 3 节给出 19 项改进的详细设计与**逐处代码改动**；第 4 节报告单元测试与多轮实机验证（含计时）；第 5 节总结并展望。附录 A 给出全部修改位置的精确索引，附录 B 给出提交记录与第一轮完整 unified diff。

## 2 系统现状

MCC 寻路由两个模块构成：

1. **规划层** `MinecraftClient/Mapping/Movement.cs`：A* 在方块网格上搜索。邻居生成函数 `GetAvailableMoves` 基于 `CanMove`（身体是否放得下）与 `IsSafe`（岩浆等伤害、坠落高度 ≤3 格、禁水）过滤；`ReconstructPath` 将节点回溯为方块中心路点。
2. **执行层** `MinecraftClient/McClient.cs`：`UpdatePathfindingInput` 每 tick（20 TPS）将路点转化为 `MovementInput`（Forward/Jump/Sneak/Sprint），由 `PlayerPhysics`（原版 1.21.11 物理）积分并发送位置包。

原有实现的问题集中体现在**规划层缺少 3D 过渡建模**（台阶、悬崖拐角）与**执行层缺少环境感知**（跳跃时机、边缘、落地判定）。

## 3 方法：19 项改进与代码改动

> 注：3.1–3.9 各小节标注的代码行号以基线 `7b100a31` 为准；3.10–3.15 标注的是当前 `master`（`7d4e3e72`）行号。全部现行号以附录 A 为准。

### 3.1 路径平滑（直线化）——修复 P1、P6

**位置**：`MinecraftClient/Mapping/Movement.cs`：`CanTravelStraight`（新增，L312-355）、`SimplifyPath`（新增，L357-391）、`IsWalkableColumn`（新增，L393-410）、`CalculatePath` 返回值接入（L267、L297）。

**原理**：A* 输出的逐格中心折线在开阔地形上产生锯齿。改进后，对重建路径做贪心视线裁剪：从当前锚点出发，若到候选路点的直线段在玩家碰撞盒（半宽 0.3 格）下全程可通行，则跳过中间路点；否则保留前一路点并继续。`CanTravelStraight` 每 0.25 格采样，检查线段所触及的所有方块列；未加载区块视为不可达。

**代码改动（新增）**：

```csharp
/// 直线可达性：同平面线段 + 玩家碰撞盒（0.6 宽）逐点采样
public static bool CanTravelStraight(World world, Location from, Location to, double playerRadius = 0.3)
{
    if (Math.Abs(from.Y - to.Y) > 0.01)
        return false; // Smoothing is limited to same-plane segments for now
    double dx = to.X - from.X, dz = to.Z - from.Z;
    double distance = Math.Sqrt(dx * dx + dz * dz);
    if (distance < 1e-6) return true;
    int steps = Math.Max(1, (int)Math.Ceiling(distance / 0.25));
    for (int i = 0; i <= steps; i++)
    {
        double t = (double)i / steps;
        double px = from.X + dx * t, pz = from.Z + dz * t;
        int minX = (int)Math.Floor(px - playerRadius), maxX = (int)Math.Floor(px + playerRadius);
        int minZ = (int)Math.Floor(pz - playerRadius), maxZ = (int)Math.Floor(pz + playerRadius);
        for (int blockX = minX; blockX <= maxX; blockX++)
            for (int blockZ = minZ; blockZ <= maxZ; blockZ++)
                if (!IsWalkableColumn(world, new Location(blockX, from.Y, blockZ)))
                    return false;
    }
    return true;
}

/// 贪心视线裁剪：能直走就合并路点，切角撞方块则保留拐角
public static Queue<Location> SimplifyPath(World world, Queue<Location> path)
{
    if (path.Count <= 2) return path;
    List<Location> waypoints = path.ToList();
    List<Location> simplified = new() { waypoints[0] };
    int anchorIndex = 0;
    for (int candidateIndex = 1; candidateIndex < waypoints.Count; candidateIndex++)
    {
        if (!CanTravelStraight(world, waypoints[anchorIndex], waypoints[candidateIndex]))
        {
            int keptIndex = candidateIndex - 1;
            if (keptIndex > anchorIndex) { simplified.Add(waypoints[keptIndex]); anchorIndex = keptIndex; }
        }
    }
    if (simplified.Last() != waypoints[^1]) simplified.Add(waypoints[^1]);
    return new Queue<Location>(simplified);
}

/// 平滑采样列必须"3 格内有地面支撑"，否则会把路径压平跨过缺口（P6）
private static bool IsWalkableColumn(World world, Location feet)
{
    ChunkColumn? chunkColumn = world.GetChunkColumn(feet);
    if (chunkColumn is null || !chunkColumn.FullyLoaded) return false;
    if (world.GetBlock(feet).Type.IsSolid()
        || world.GetBlock(Move(feet, Direction.Up)).Type.IsSolid())
        return false;
    return world.GetBlock(Move(feet, Direction.Down)).Type.IsSolid()
        || world.GetBlock(Move(feet, Direction.Down, 2)).Type.IsSolid()
        || world.GetBlock(Move(feet, Direction.Down, 3)).Type.IsSolid()
        || IsClimbing(world, Move(feet, Direction.Down))
        || IsClimbing(world, Move(feet, Direction.Down, 2))
        || IsClimbing(world, Move(feet, Direction.Down, 3));
}
```

**接入点**：`CalculatePath` 的两处 `return ReconstructPath(...)` 改为 `return SimplifyPath(world, ReconstructPath(...))`。

### 3.2 3D 邻域建模：斜上跨步（Step-up）——修复 P2

**位置**：`Movement.cs`：`GetAvailableMoves`（L55-93）、`CanStepUp`（新增，L126-150）、`CanMove` 的 `Direction.Up` 分支（L755-770）、`IsSafe`（L716-745）。

**原理**：原移动生成器在同一列上生成 `Up` 移动，但"原地直跳"的目标脚方块上方是空气、下方无支撑，物理上不可能落地。本文新增**斜上跨步**：当相邻格存在 1 格高实心方块（台阶/坑沿）且目标格（上方一格）身体可容纳时，生成 `(Δx, +1, Δz)` 的斜上节点。同时将同列 `Up` 收紧为仅允许攀爬（梯子/藤蔓）或游泳。

**代码改动（GetAvailableMoves，核心新增）**：

```csharp
// 地面/游泳分支：原代码只遍历 8/10 方向做水平移动
foreach (Direction dir in Enum.GetValues(typeof(Direction)))
{
    Location dest = Move(location, dir);
    bool diagonalSafe = !IsDiagonal(dir)
        || (IsSafe(world, Move(location, DiagonalCardinalA(dir)))
            && IsSafe(world, Move(location, DiagonalCardinalB(dir))));
    if (CanMove(world, location, dir) && (allowUnsafe || (IsSafe(world, dest) && diagonalSafe)))
        availableMoves.Add(dest);
}

// 新增：斜上跨步（1 格高台阶）
foreach (Direction dir in new[] { Direction.East, Direction.West, Direction.North, Direction.South })
{
    Location stepUp = Move(Move(location, dir), Direction.Up);
    if (CanStepUp(world, location, dir) && (allowUnsafe || IsSafe(world, stepUp)))
        availableMoves.Add(stepUp);
}

/// 台阶判定：相邻格是 1 格高实心方块，且目标格（上方）身体放得下
private static bool CanStepUp(World world, Location location, Direction direction)
{
    if (!IsOnGround(world, location) && !IsSwimming(world, location)) return false;
    Location stepBlock = Move(location, direction);
    Location destination = Move(stepBlock, Direction.Up);
    if (!world.GetBlock(stepBlock).Type.IsSolid()) return false;
    if (world.GetBlock(destination).Type.IsSolid()
        || world.GetBlock(Move(destination, Direction.Up)).Type.IsSolid())
        return false;
    return true;
}
```

**代码改动（CanMove 的 Up 分支，修改）**：

```csharp
case Direction.Up:
    // 原实现：ground 情况下仅检查 y+2 非实心 → 允许"原地直跳进空气"
    // 修改后：同列上升仅限攀爬或游泳；台阶/出坑走斜上跨步
    if (IsClimbing(world, location))
        return IsClimbing(world, Move(location, Direction.Up))
            || (!world.GetBlock(Move(location, Direction.Up)).Type.IsSolid()
                && !world.GetBlock(Move(Move(location, Direction.Up), Direction.Up)).Type.IsSolid());
    return IsSwimming(world, location)
        && !world.GetBlock(Move(location, Direction.Up)).Type.IsSolid()
        && !world.GetBlock(Move(Move(location, Direction.Up), Direction.Up)).Type.IsSolid();
```

**代码改动（IsSafe，修改）**：增加"目标脚方块必须非实心"：

```csharp
return
    !world.GetBlock(location).Type.IsSolid()          // 新增：目标脚方块可通行
    && !world.GetBlock(location).Type.CanHarmPlayers()
    && !world.GetBlock(Move(location, Direction.Up)).Type.CanHarmPlayers()
    && !world.GetBlock(Move(location, Direction.Down)).Type.CanHarmPlayers()
    // ...坠落高度、禁水检查不变
```

### 3.3 斜向移动安全约束——修复 P7

**位置**：`Movement.cs`：`GetAvailableMoves`（L65-68）、`IsDiagonal`/`DiagonalCardinalA`/`DiagonalCardinalB`（新增，L95-124）。

**原理**：安全模式下，斜向移动（NE/SE/SW/NW）除目标格外，还要求相邻的两个直角格均安全。原实现只检查身体放得下，导致斜穿 1 格宽平台的悬崖拐角时扫入深坑。

**代码改动（新增辅助）**：

```csharp
private static bool IsDiagonal(Direction direction)
{
    return direction == Direction.NorthEast || direction == Direction.SouthEast
        || direction == Direction.SouthWest || direction == Direction.NorthWest;
}

private static Direction DiagonalCardinalA(Direction direction) => direction switch
{
    Direction.NorthEast => Direction.North,
    Direction.SouthEast => Direction.South,
    Direction.SouthWest => Direction.South,
    Direction.NorthWest => Direction.North,
    _ => throw new ArgumentException("Not a diagonal direction", nameof(direction))
};

private static Direction DiagonalCardinalB(Direction direction) => direction switch
{
    Direction.NorthEast => Direction.East,
    Direction.SouthEast => Direction.East,
    Direction.SouthWest => Direction.West,
    Direction.NorthWest => Direction.West,
    _ => throw new ArgumentException("Not a diagonal direction", nameof(direction))
};
```

### 3.4 物理一致的到达判定——修复 P3

**位置**：`MinecraftClient/McClient.cs`：`ReachedWaypoint`（L3725-3742）。

**原理**：路点位于方块中心（Y+0.5），而角色脚部位于方块顶面（Y+1）。原判定仅看水平距离，导致跳跃中途即"到达"并推进下一路点，从楼梯边缘滑落；另一版本的方向性垂直检查又因"脚比路点高 0.5 格"而永远无法到达（表现为原地看天）。最终判定为：水平距离 <0.7 格 **且** 垂直距离 <0.6 格 **且** 处于落地/攀爬/游泳状态。

**代码改动（重写）**：

```csharp
// 修改前：
// return dx * dx + dz * dz < 0.25; // within ~0.5 blocks horizontally

// 修改后：
private bool ReachedWaypoint(Location target)
{
    double dx = target.X - location.X;
    double dz = target.Z - location.Z;
    if (dx * dx + dz * dz >= 0.49)
        return false; // not within ~0.7 blocks horizontally

    // 路点在方块中心（Y+0.5），脚在方块顶（Y+1），用绝对容差 ±0.6
    double dy = target.Y - location.Y;
    if (Math.Abs(dy) >= WaypointVerticalTolerance)
        return false;

    // 必须真正落地/攀爬/游泳：跳跃中途不推进，先站稳台阶顶端
    return playerPhysics.OnGround
        || playerPhysics.InWater
        || playerPhysics.OnClimbable
        || playerPhysics.VerticalCollisionBelow;
}
```

### 3.5 卡住检测与自动重规划——修复 P5

**位置**：`McClient.cs`：`UpdatePathfindingInput`（L3578-3652）、`ResetPathProgress`（新增，L3658-3666）、`ReplanMovement`（新增，L3670-3720）、`MoveTo`（L1785-1810）、`CancelMovement`（L3883-3889）、状态字段与常量（L86-97）。

**原理**：落地状态下水平距离 3 秒无进展即判定卡住，从当前位置以 2 秒超时重算到原始目标的路径；只有"距目标净前进 >1 格"才重置重规划计数，连续 5 次无进展则取消移动并输出提示。

**代码改动（状态字段，新增）**：

```csharp
private Location? movementGoal; // 原始目标，供卡住后重规划
private bool movementAllowUnsafe;
private int movementMaxOffset;
private int movementMinOffset;
private double lastWaypointDistanceSqr = double.MaxValue;
private double lastReplanGoalDistanceSqr = double.MaxValue;
private int pathStuckTicks;
private int replanCount;
private const int PathStuckThresholdTicks = 60;   // ~3 秒 @20TPS
private const int MaxReplansWithoutProgress = 5;
private const double PathProgressEpsilonSqr = 0.0025; // ~0.05 格
private const double WaypointVerticalTolerance = 0.6;
```

**代码改动（UpdatePathfindingInput 卡住检测，新增）**：

```csharp
if (pathTarget is not null)
{
    double dx = pathTarget.Value.X - location.X;
    double dz = pathTarget.Value.Z - location.Z;
    double distSqr = dx * dx + dz * dz;
    if (distSqr < lastWaypointDistanceSqr - PathProgressEpsilonSqr)
    {
        lastWaypointDistanceSqr = distSqr;
        pathStuckTicks = 0;
    }
    else if (playerPhysics.OnGround && pathStuckTicks > PathStuckThresholdTicks)
    {
        ReplanMovement();
        return;
    }
    else if (playerPhysics.OnGround)
    {
        // 只在落地时计时：滞空是正常的跳跃/攀爬
        pathStuckTicks++;
    }
    SetInputToward(pathTarget.Value);
}
```

**代码改动（ReplanMovement，新增）**：

```csharp
private void ReplanMovement()
{
    if (movementGoal is not Location goal) { path = null; pathTarget = null; return; }

    double goalDistanceSqr = goal.DistanceSquared(location);
    if (goalDistanceSqr < lastReplanGoalDistanceSqr - 1.0)
    {
        // 距目标净前进 >1 格：重规划有效，重置计数
        lastReplanGoalDistanceSqr = goalDistanceSqr;
        replanCount = 0;
    }
    if (++replanCount > MaxReplansWithoutProgress)
    {
        path = null; pathTarget = null; movementGoal = null; replanCount = 0;
        ConsoleIO.WriteLineFormatted("§c[MCC] Movement cancelled: cannot escape the current position after repeated retries. Run /move again.");
        return;
    }

    Queue<Location>? newPath = Movement.CalculatePath(
        world, location, goal, movementAllowUnsafe, movementMaxOffset, movementMinOffset,
        TimeSpan.FromSeconds(2));
    pathTarget = null;
    path = newPath;
    if (newPath is null || newPath.Count == 0)
    {
        path = null; movementGoal = null;
        ConsoleIO.WriteLineFormatted("§c[MCC] No recovery path found; movement cancelled. Run /move again.");
        return;
    }
    pathTarget = newPath.Dequeue();
    ResetPathProgress(pathTarget.Value);
}
```

**代码改动（MoveTo，修改）**：保存目标与参数，供重规划使用；`CancelMovement` 同步清理 `movementGoal`。

### 3.6 原版风格自动跳跃——修复 P3 的执行侧

**位置**：`McClient.cs`：`SetInputToward`（L3791-3811）。

**原理**：不再"目标在上方就跳"（早期方案会冲刺跳，冲过头摔下平台），改为探测正前方 0.7 格：存在 1 格高实心台阶且上方两格可通行时，轻跳一步（不冲刺）。

**代码改动（替换原 dy 跳跃判定）**：

```csharp
// 修改前：if (dy > 0.5 && playerPhysics.OnGround) physicsInput.Jump = true;

// 修改后：原版风格自动跳跃
if (playerPhysics.OnGround && physicsInput.Forward)
{
    double yawRad = playerYaw * (Math.PI / 180.0);
    double forwardX = -Math.Sin(yawRad);
    double forwardZ = Math.Cos(yawRad);
    Location aheadBlock = new(
        Math.Floor(location.X + forwardX * 0.7),
        Math.Floor(location.Y),
        Math.Floor(location.Z + forwardZ * 0.7));
    if (world.GetBlock(aheadBlock).Type.IsSolid()
        && !world.GetBlock(Movement.Move(aheadBlock, Direction.Up)).Type.IsSolid()
        && !world.GetBlock(Movement.Move(aheadBlock, Direction.Up, 2)).Type.IsSolid())
    {
        physicsInput.Jump = true;
    }
}
```

### 3.7 边缘探测——修复 P4

**位置**：`McClient.cs`：`SetInputToward`（L3813-3848）。

**原理**：落地前进时探测前方格子，若其 2 格内无地面支撑（悬崖）则停止迈步；1-2 格下落允许（物理可处理），深坑一律阻挡。

**代码改动（新增）**：

```csharp
if (playerPhysics.OnGround && physicsInput.Forward)
{
    double yawRad = playerYaw * (Math.PI / 180.0);
    double forwardX = -Math.Sin(yawRad);
    double forwardZ = Math.Cos(yawRad);
    Location currentCell = new(Math.Floor(location.X), Math.Floor(location.Y), Math.Floor(location.Z));
    Location aheadCell = new(
        Math.Floor(location.X + forwardX * 0.7),
        Math.Floor(location.Y),
        Math.Floor(location.Z + forwardZ * 0.7));
    if (aheadCell != currentCell
        && !world.GetBlock(aheadCell).Type.IsSolid()
        && !world.GetBlock(Movement.Move(aheadCell, Direction.Down)).Type.IsSolid()
        && !world.GetBlock(Movement.Move(aheadCell, Direction.Down, 2)).Type.IsSolid())
    {
        physicsInput.Forward = false; // 前方 2 格内无地面 → 悬崖，停步
    }
}
```

### 3.8 1 格宽路径居中校正——P7 执行侧补充

**位置**：`McClient.cs`：`SetInputToward`（L3758-3788）、`IsCliffSide`（新增，L3851-3862）。

**原理**：当当前格东西两侧（或南北两侧）均为悬崖（即 1 格宽走廊/平台）时，将行进方向与"回到格子中心"的修正向量按 2.5 倍强度混合，使机器人始终沿中心线行走。

**代码改动（新增）**：

```csharp
// 在计算朝向之前，叠加居中修正
double steerX = 0, steerZ = 0;
if (playerPhysics.OnGround)
{
    double centerX = Math.Floor(location.X) + 0.5;
    double centerZ = Math.Floor(location.Z) + 0.5;
    double offX = centerX - location.X;
    double offZ = centerZ - location.Z;
    if (Math.Abs(offX) > 0.1
        && IsCliffSide(world, location, Direction.East)
        && IsCliffSide(world, location, Direction.West))
        steerX = offX * 2.5;
    if (Math.Abs(offZ) > 0.1
        && IsCliffSide(world, location, Direction.North)
        && IsCliffSide(world, location, Direction.South))
        steerZ = offZ * 2.5;
}
float targetYaw = (float)(-Math.Atan2(dx + steerX, dz + steerZ) / Math.PI * 180.0);

/// 悬崖判定：相邻格非实心且 2 格内无地面
private bool IsCliffSide(World world, Location location, Direction direction)
{
    Location side = Movement.Move(location, direction);
    if (world.GetBlock(side).Type.IsSolid()) return false;
    return !world.GetBlock(Movement.Move(side, Direction.Down)).Type.IsSolid()
        && !world.GetBlock(Movement.Move(side, Direction.Down, 2)).Type.IsSolid();
}
```

### 3.9 附带的健壮性修正

- `Movement.cs` `PlayerFitsHere`：`McClient.Instance` 空值安全（`int? protocolVersion = McClient.Instance?.GetProtocolVersion();`），使寻路逻辑可在无连接环境下被单元测试。
- `McClient.cs`：`MoveTo` 增加路径诊断输出（`[MCC] Path (N): ...`），便于实机调试。

### 3.10 搜索层重构：JPS 跳点搜索与真实距离启发式——修复 P8

**位置**：`Movement.cs`：`GetJumpPointNeighbors`（L538）、`Jump`/`JumpRecursive`（L586-660）、`CanWalkHorizontally`（L664）、`IsLoaded`/`IsOpenCell`（L676-686）、`HasForcedNeighbor`（L690）、`MoveCost`/`Heuristic`（L505-535）、常量 `MoveCostScale = 1000`（L22）。

**原理**：第一代 `GetAvailableMoves` 对每个节点枚举 8/10 个邻居，在大型迷宫（Backrooms 一类楼层结构）中搜索爆炸。第二代将其替换为 **JPS（Jump Point Search）**：

1. **横向 8 方向跳点**：`JumpRecursive` 沿方向连续前进，只在目标格、强制邻居（`HasForcedNeighbor`）、垂直过渡点、悬空/未加载边缘格处停止并产出跳点；开阔走廊被压缩为首尾两个跳点，中间格不再入队。
2. **垂直过渡**：斜上跨步（1 格高台阶）、走下边缘（进入无支撑格开始下落）、攀爬/游泳同列上行、空中继续下落，保证 3D 地形仍可规划。
3. **真实距离启发式**：`Heuristic` 与 `MoveCost` 使用 `MoveCostScale × 欧氏距离`。旧实现用平方距离做 H（26 格远的目标评 712 而不是 ~26），严重高估使 A* 变成贪心搜索；替换后同一迷宫展开节点从 **40 万+ 降到 1~3 千**，搜索时间从超时降到毫秒级。

**代码改动（关键新增）**：

```csharp
private static int Heuristic(Location from, Location to)
{
    double dx = Math.Abs(to.X - from.X);
    double dy = Math.Abs(to.Y - from.Y);
    double dz = Math.Abs(to.Z - from.Z);
    return (int)Math.Round(MoveCostScale * Math.Sqrt(dx * dx + dy * dy + dz * dz));
}

private static Location? JumpRecursive(World world, Location next, Direction dir, Location goal)
{
    if (!PlayerFitsHere(world, next))
        return null;                       // 身体放不下：终止且不产出跳点
    if (next == goal)
        return next;
    if (!IsWalkableColumn(world, next))
        return next;                       // 悬空/未加载边缘：作为跳点交给下落过渡
    if (HasForcedNeighbor(world, next, dir))
        return next;                       // 障碍导致的强制邻居
    if (IsDiagonal(dir)
        && (JumpRecursive(world, Move(next, DiagonalCardinalA(dir)), DiagonalCardinalA(dir), goal) != null
            || JumpRecursive(world, Move(next, DiagonalCardinalB(dir)), DiagonalCardinalB(dir), goal) != null))
        return next;
    if (HasVerticalTransition(world, next))
        return next;                       // 可斜上/攀爬/下落的位置必须停下评估
    if (!CanWalkHorizontally(world, next, dir))
        return null;
    return JumpRecursive(world, Move(next, dir), dir, goal);
}
```

### 3.11 双向 A* 搜索

**位置**：`Movement.cs`：`CalculatePathBidirectional`（L343-463）、`GetReverseNeighbors`（L471-505）、`PriorityQueue`。

**原理**：`-f` 风险模式且未指定偏移时启用双向搜索：正向从起点用 JPS 生成邻居，反向从目标用 `GetReverseNeighbors` 生成前驱（水平反向、斜上跨步反向、下落反向），双方共享真实距离启发式，在相遇点汇合后拼接路径。`PriorityQueue` 采用 .NET 内置优先队列 + 惰性删除（`entryG != entry.G` 跳过过期条目），替代原手写二叉堆。反向邻居与正向共享同一支撑规则，避免反向搜索把正向不可达的虚空格引入路径（详见 3.13）。

**实测效果**：Backrooms 起点到目标的首次搜索约 2 万节点（65 路点），重规划约 3-6 万节点，均远低于旧 A* 的 40 万+。

### 3.12 异步路径搜索

**位置**：`McClient.cs`：`MoveTo`（L1797）、`UpdatePathfindingInput` 的任务接管分支（L3603-3637）、`ReplanMovement`（L3751）、`CancelPendingPathSearch`。

**原理**：搜索改为后台 `Task.Run`，`pathSearchCts.CancelAfter(120s)` 限时；`MoveTo` 立即返回 "Searching..."，主循环每 tick 检查任务完成并接管路径；卡住触发的重规划同样异步，不再阻塞 20 TPS 物理循环。`CancelMovement` 同步取消未完成任务，避免旧结果覆盖新移动。

### 3.13 `-f` 风险模式：虚空拦截与"服务器地形信任"——修复 P9

**位置**：`Movement.cs`：`HasSupportWithin`（L848-868）、`UnsafeFallDepth = 5`（L27）、`GetJumpPointNeighbors` 走下边缘分支（L550-562）、`GetReverseNeighbors` 两处镜像检查；`McClient.cs`：`SetInputToward` 边缘探测（L3950-3983）。

**背景**：Backrooms 实测发现两个对偶问题：

1. **寻路器把无底虚空当可行走**：`-f` 原实现 `allowUnsafe || IsSafe(...)` 对"已加载且脚下 3 格内无任何支撑"的格子直接放行，机器人走进虚空后触发服务器反虚空插件每 tick 拉回 y=19，表现为原地"小碎步"。
2. **执行器把真实地板当悬崖**：对比服务器存档（解析 `esY - Copy` 世界的 `r.-1.-1.mca`，坐标 (-177/-178, 18, -308) 均为 `yellow_concrete_powder`）与 MCC 客户端区块缓存（同一坐标返回 Air），证实客户端缓存与服务器不一致——blockinfo 对未加载/缺失区块返回 Air，无法区分"真虚空"与"缓存缺失"。

**修复**：

- 规划层：`HasSupportWithin(world, feet, depth)` 在 `-f` 下要求目标格下方 5 格内有实心/可攀爬支撑，**未加载区块视为未知并放行**（探索语义）；已加载的无底虚空列不再生成边。反向邻居镜像同一规则，防止双向搜索在非法格相遇。
- 执行层：`-f` 时**完全跳过客户端缓存边缘探测**，只靠服务器物理、卡住重规划与反虚空插件兜底；安全模式保留 3 格探测（与 `IsSafe` 一致）。

```csharp
private static bool HasSupportWithin(World world, Location feet, int depth)
{
    if (!IsLoaded(world, feet))
        return true; // 未知区块：不是可证明的虚空
    for (int d = 1; d <= depth; d++)
    {
        Location below = Move(feet, Direction.Down, d);
        if (!IsLoaded(world, below))
            return true;
        if (world.GetBlock(below).Type.IsSolid() || IsClimbing(world, below))
            return true;
    }
    return false;
}
```

### 3.14 路径执行朝向接管——修复 P10 的主因

**位置**：`McClient.cs` 主循环（L706-738）、`SetInputToward` 尾部（L3916-3925）。

**根因**：服务器每 ~20 tick 发送位置包，`Protocol18.cs` 在 L1601/1614 调用 `handler.UpdateLocation(location, yaw, pitch)` 把 `_yaw` 钉为服务器端旧朝向；`yaw=0` 是**合法非 null** 值，主循环原有 `if (_yaw is not null) playerPhysics.Yaw = _yaw.Value;` 每 tick 把移动朝向覆盖回 0（正南），而 `SetInputToward` 算出的路点朝向（如 90/128）被抵消。叠加 `MoveHeadWhileWalking` 的路点注视钉扎，机器人表现为 x 轴完全不动、沿 z 缓慢漂移的"小碎步"。

**修复**：路径活跃期间（`pathTarget` 或后台搜索非空）完全忽略 `_yaw/_pitch` 钉扎，且 `SetInputToward` 每 tick 无条件清除 `_yaw/_pitch`，让物理引擎的修正后朝向真正驱动移动：

```csharp
bool pathActive = pathTarget is not null || pathSearchTask is not null;
if (!pathActive)
{
    if (_yaw is not null) playerPhysics.Yaw = _yaw.Value;
    if (_pitch is not null) playerPhysics.Pitch = _pitch.Value;
}
// ...
playerYaw = !pathActive && _yaw is not null ? _yaw.Value : playerYaw;
```

### 3.15 墙碰撞定点解卡与重规划计数修复——修复 P10 的次要因素

**位置**：`McClient.cs`：字段 `wallUnstickTicks`/`wallUnstickYaw`（L97-98）、`SetInputToward` 碰撞分支（L3868-3903）、`ReplanMovement`（L3758-3769）、`MoveTo` 初始化（L1816）。

**问题 1：解卡差半步**。碰撞时原先"向格中心叠加小修正"，但修正量被远处路点的 atan2 方向稀释，且每 tick 重新评估——碰撞一闪失就恢复顶墙，机器人永远差 0.05~0.1 格无法脱身（实测卡在 (-214.70, -305.27)，距墙缘 0.07 格）。修复：碰撞触发后**锁定解卡朝向 12 tick**（约 0.6 秒），朝当前格中心连续移动；若已在格中心仍碰撞（夹缝），则改走路点方向的垂直侧向。解卡期间跳过路点朝向与边缘探测，保证一次走完脱困距离。

**问题 2：重规划计数永不归零逻辑**。原实现用平方距离判断"距目标净前进 >1 格"：目标很远时，脚边 0.15 格抖动即可让距离平方变化 10+（杠杆效应），被误判为有效进展，`replanCount` 永远重置，卡死的机器人无限重规划（实测连续 100+ 次、每次 20-60 万节点）。修复：改用线性距离 `goal.Distance(location)`，比较阈值为 1.0 格，连续 5 次重规划无净进展即取消并提示。

### 3.16 爬梯执行层：碰撞、居中、顶翻越与跳跃

**位置**：`PlayerPhysics.cs`：`HandleJumping`（L140-158）、`TravelInAir` 爬升 bump（L206-220）；`CollisionDetector.cs`：`CollectBlockColliders`（L153）；`McClient.cs`：`SetInputToward` 爬梯模式（L3850-3935）、`stuck` 检测爬梯豁免（L3734）。

**问题链**（实机逐层暴露）：

1. **梯子被当作实心方块**：`MaterialExtensions.IsSolid` 的 case 列表包含 `Ladder`，导致机器人在梯子格内被梯子方块本身碰撞——垂直爬升时**头顶的梯子块挡住上升**（实测卡在 y=20.20，vel 显示 +0.118 却无法位移）。
2. **爬升依赖水平碰撞**：原版语义"贴墙（水平碰撞）时 bump 上爬"，梯子改为非碰撞后，居中站在梯子格中心的机器人碰不到任何墙，永远不产生爬升速度。
3. **梯子边缘滑脱**：机器人从梯子格边缘进入时，身体只有一部分在梯子列内，爬升中会被挤向侧面（用户观察为"爬一格子的梯子要居中"）。
4. **梯子顶无法翻越**：爬出梯子顶格后 `OnClimbable` 消失，机器人悬空、不水平移动，滑回梯子中段反复循环；且物理引擎**在梯子上不允许跳跃**（`HandleJumping` 仅当 `OnGround`），无法跳出梯子顶到平台。

**修复**：

```csharp
// 1) CollisionDetector：可攀爬方块不产生碰撞盒
Block block = world.GetBlock(new Location(bx, by, bz));
if (block.Type.CanBeClimbedOn())
    continue; // ladders/vines are passable; climbing pushes against the wall behind

// 2) PlayerPhysics：在梯子上且有移动输入就产生爬升速度（不依赖水平碰撞）
bool wantsToMove = Xxa != 0.0f || Zza != 0.0f;
if (OnClimbable && (HorizontalCollision || Jumping || wantsToMove))
    vy = PhysicsConsts.ClimbWallBump;

// 3) HandleJumping：梯子上允许跳跃（原版行为，用于跳出梯子顶）
else if ((OnGround || OnClimbable) && noJumpDelay == 0)
{
    JumpFromGround();
    noJumpDelay = 10;
}
```

执行层爬梯模式（`SetInputToward`，位于普通航向计算之前）：

```csharp
// a) 先走到梯子格中心（偏差 >0.15 格时朝中心走），再贴墙爬
double centerX = ladder.X + 0.5 - location.X;
double centerZ = ladder.Z + 0.5 - location.Z;
if (centerX * centerX + centerZ * centerZ > 0.0225) { /* 朝中心走并 return */ }
// b) 已居中：面向最近的实心邻格（梯子背后的墙），W 推墙触发爬升
// c) 梯子顶：feet 不再是梯子但脚下 1-2 格是梯子 → 朝路点方向 Forward + Jump 跳出
bool belowLadder = world.GetBlock(Move(feetCell, Direction.Down)).Type.CanBeClimbedOn()
    || world.GetBlock(Move(feetCell, Direction.Down, 2)).Type.CanBeClimbedOn();
if (belowLadder) { physicsInput.Forward = true; if (dy > 0.5) physicsInput.Jump = true; return; }
```

同时 `stuck` 检测在 `OnClimbable` 时不再计数：爬梯时到路点的水平距离几乎不变，计数会让机器人 3 秒触发重规划、清空 `pathTarget`，随后服务器位置包重新生效并把机器人拉回梯子底部（见 3.17）。

**实测效果**：第一架 4 格梯子（y=19→22）完整爬升并翻上 y=23 平台（日志：y 20.06 → 22.55 → 23.00）；第二架梯子（y=19→22）同样成功，机器人随后沿 y=22/23 平台完成 115 格冲刺。

### 3.17 联机位置同步与未知地面

**位置**：`McClient.cs`：`UpdateLocation(Location, float, float)`（L4182-4197）；`PlayerPhysics.cs`：`HasUnknownGround`（L~597）、`TravelInAir` 重力分支（L226-235）、`Move()` 的 `OnGround` 修正（L~371）。

**问题 1：服务器位置回显清零速度**。MCC 客户端每 tick 向服务器发送位置包，服务器以 `PlayerPositionAndLook` 回显（`Protocol18.cs` L1601/1614 → `handler.UpdateLocation(location, yaw, pitch)` → `Teleport`）。`PlayerPhysics.Teleport` 会**清零 DeltaMovement**。路径执行中，服务器回显的旧位置每 tick 把机器人 Teleport 回上一已知点，爬升速度与水平修正被清零——实测机器人 y 被钉在 19.35/19.60 反复"小碎步"；且一旦 `ReplanMovement` 清空 `pathTarget`，本忽略逻辑失效，机器人直接掉回梯子底部。

**修复**：路径活跃（`pathTarget` 或后台搜索非空）且服务器位置与本地物理差距 <2 格时，**忽略回显位置**（本地物理是权威）；传送/重生/大差距（≥2 格）仍强制同步：

```csharp
bool pathActive = pathTarget is not null || pathSearchTask is not null;
bool farFromPhysics = physicsInitialized
    && new Location(playerPhysics.Position.X, playerPhysics.Position.Y, playerPhysics.Position.Z)
        .DistanceSquared(location) >= 4.0;
if (!pathActive || !physicsInitialized || farFromPhysics)
    UpdateLocation(location, false);
```

**问题 2：未加载区块导致物理悬空**。`World.GetBlock` 对未加载 chunk 返回 Air，机器人站在"服务器有地板但客户端缓存缺失"的区域时被物理判定为悬空，每 tick 下落一点、被服务器反虚空插件拉回 y=19，水平速度反复清零。

**修复**：物理层引入"未知地面"语义（与寻路层 `Movement.IsOnGround` 对未加载 chunk 返回 true 的规则一致）——脚下 3 格所在 chunk 未完全加载时：跳过重力、`Move()` 后强制 `OnGround = true`，机器人以正常步行速度进入正在流式加载的地形；已加载的真实虚空仍正常下落。

```csharp
private bool HasUnknownGround(World world)
{
    Location feet = new(Math.Floor(Position.X), Math.Floor(Position.Y), Math.Floor(Position.Z));
    for (int d = 1; d <= 3; d++)
    {
        Location below = Movement.Move(feet, Direction.Down, d);
        ChunkColumn? column = world.GetChunkColumn(below);
        if (column is null || !column.FullyLoaded)
            return true;
    }
    return false;
}
```

### 3.18 搜索预算与重规划计数

**位置**：`McClient.cs`：`MoveTo`（L1821）、`ReplanMovement`（L3802）、异步任务完成分支（L3621-3630）。

**问题 1：远目标搜索超时**。目标 (-416, 28) 距起点 240+ 格且大部分区块未加载，JPS 在未知区域几乎无法剪枝，首次搜索需 159 万节点/约 5 分钟。原 120 秒超时导致 67 万节点时直接失败。修复：`CancelAfter` 默认与重规划超时均放宽到 **300 秒**。

**问题 2：重规划取消永不触发**。`ReplanMovement` 的"连续 5 次无进展取消"计数在**每次后台搜索成功完成时被无条件清零**（任务完成分支 `replanCount = 0`），卡死机器人每 3 秒重规划一次、节点数无上限增长（实测 26 万→45 万→继续）。修复：删除任务完成分支的清零，仅 `MoveTo` 新命令时重置计数。修复后卡死机器人在 5 次重规划后输出 "Movement cancelled" 并停止，不再无限烧 CPU。

### 3.19 路上规划（分段导航）与实体环境适配

**位置**：`Movement.cs`：`CalculatePath` 节点预算（L242/L298/L380）、`CalculatePathBidirectional` 部分路径返回（L465）、`HasBlockData`（L701）；`McClient.cs`：`StartPathSearch`（L1838）、分段续段（L3706-3712）、服务器回弹检测（L3781）、区块等待豁免（L3803）、`IsAheadChunkLoading`（L3822）。

**问题**：目标 (-416, 28) 距起点 240+ 格，大部分区块未加载。一次性搜索需要 159 万节点/约 5 分钟且可能超时；更糟的是**未知区域可能被误判**——客户端"整列标记已加载但 section 数据缺失"的格子返回 Air，寻路器把真实地形当虚空拦截，或把虚空当可行走进服务器实体边界被拉回（用户观察为"区块没加载到那个地方""bot 在梯子旁边"）。

**修复（四层）**：

1. **节点预算与部分路径**：`CalculatePath` 增加 `maxNodes` 预算（首段 40 万）。预算耗尽时，双向 A* 返回"正向已展开的、离目标最近的节点"到起点的前缀路径（`bestProgress`），bot 立即出发，边走边触发服务器加载前方区块。
2. **自动分段续段**：路径终点到达后，若未到最终目标（`movementFinalGoal` 距离 >1 格）且段数 <20，自动以当前位置为起点发起下一段搜索；续段预算翻倍（80 万）。实测日志：`Segment 1/20 reached; re-planning toward final goal...`。
3. **服务器回弹检测**：服务器是实体权威——当客户端物理尝试前进但位置被服务器拉回（`distSqr` 相对历史最佳突然增大 >0.5），累计 30 tick（1.5 秒）即触发重规划，避免"缓慢推进-快速回弹"的推墙循环（该循环会让普通 stuck 计数永远被"推进突破"清零）。
4. **section 级未知感知**：`HasBlockData` 检查具体 Y section 是否存在（而非仅 `column.FullyLoaded`）。寻路器 `HasSupportWithin`、物理 `HasUnknownGround`、执行层 `IsAheadChunkLoading` 三处统一：section 缺失 = 未知 = 放行/等待，已加载真虚空才拦截/下落。前方区块等待最长 60 秒（`PathChunkWaitMaxTicks = 1200`），超过仍无进展才重规划。

**实测**：起点 → (-416, 28) 任务中，首段 28 万节点/约 2 分钟返回部分路径，bot 立即开走；依次穿过 -270/-292/-306 走廊、爬第二架梯子（y 19→22），推进到 -331 第三架梯子，自动续段触发。相比之前"5 分钟算完或直接失败"，**首段等待从 5 分钟降到约 2 分钟，且不再因目标过远而整体失败**。

## 4 实验与验证

### 4.1 单元测试

新增 `MinecraftClient.Tests/MovementTests.cs`（10 个用例）：

| 测试 | 验证点 |
| --- | --- |
| `CanTravelStraight_OpenGround_ReturnsTrue` | 开阔地直线可达 |
| `CanTravelStraight_WallOnSegment_ReturnsFalse` | 线段中墙阻挡 |
| `SimplifyPath_StraightLine_CollapsesToEndpoints` | 直线合并为首尾 |
| `SimplifyPath_OpenDiagonal_CollapsesToEndpoints` | 斜线合并 |
| `SimplifyPath_LCorner_KeepsCornerWaypoint` | 拐角保留、直道合并 |
| `SimplifyPath_NarrowCorridor_DoesNotCutThroughWall` | 窄走廊不切墙 |
| `CalculatePath_StraightCorridor_ReturnsSmoothedPath` | 端到端平滑 |
| `CanMove_UpIntoSolidFeetBlock_ReturnsFalse` | 目标脚方块实心拒绝 |
| `CalculatePath_OneBlockPit_CanJumpOut` | 1 格坑可跳出 |
| `CalculatePath_OneBlockStep_CanClimb` | 1 格台阶可攀爬 |

**结果**：`dotnet test` 全套 **48/48 通过**（38 个既有 + 10 个新增）。

### 4.2 实机验证

环境：本机 Minecraft 26.2（protocol 776）离线服务器，MCC 调试模式 + 文件输入驱动，用户自建障碍场。

**改进前**：机器人卡在楼梯底部（同列直跳无效）、爬楼梯后滑落、1 格宽平台踏空坠落、卡墙无限重规划（日志可见数十次 "Movement stuck"）。

**改进后**：机器人从 Y=112 出发，经 9 级 1 格高无护栏楼梯逐级上行至 Y=121/122，沿 1 格宽楼顶平台中心线行进，绕过深坑，最终到达目标 (32,121,-23) 并完成后续 (41,122,-15) 的导航与挖掘任务。期间偶发卡点由自动重规划恢复，连续无进展时优雅取消并提示，不再死循环。

### 4.3 Backrooms 迷宫计时验证（第二代改进）

环境：本机 Minecraft 26.2（Fabric，protocol 776）服务器，世界 `esY - Copy`（TC_5's Backrooms 迷宫），离线机器人 `pathbot`，文件输入驱动，风险模式 `move ... -f`。

**协议**：每次测试前先 `/tp -176.12 19.00 -308.32` 固定起点，再发起 `move -317 19.45 -363.2 -f`，以日志中 "Walking from ..." 为开始时刻，以机器人停靠目标附近且再次 `move` 显示 1 路点直达为到达时刻。

**结果（最终版）**：

| 指标 | 数值 |
| --- | --- |
| 起点 | (-176.12, 19.00, -308.32) |
| 终点 | (-316.65, 19.00, -362.92)（距目标 (-317, 19.45, -363.2) 0.5 格） |
| 总耗时 | **2 分 00 秒**（23:09:52 → 23:11:52） |
| 路程 | 约 160 格（含绕行） |
| 首次搜索 | 65 路点 / 20,130 展开节点 |
| 中途重规划 | 3 次（约 -214.6、-270.6、-277.4 处），每次 1-2 秒 |

**改进前对照**（同一协议）：机器人从起点出发后约 1 分钟内在起点附近（(-176.93, -308.94)）或 (-214.70, -305.27) 处陷入"小碎步"——yaw 被服务器位置包钉回 0、边缘探测把缓存中的"假虚空"当悬崖、碰撞解卡差 0.07 格，3 秒一次重规划且永不取消，数分钟无净进展。

**归因**：3.13（虚空拦截与服务器信任）、3.14（朝向接管）、3.15（定点解卡与计数修复）三者缺一不可：去掉 3.14 时机器人卡在起点；只保留 3.14 时机器人能穿过起点走廊但在 -214.70 反复顶墙；三者齐备后 2 分钟完成全程。

### 4.4 爬梯与远距离目标验证（3.16-3.18）

环境同上（Backrooms 迷宫，`-f` 风险模式）。针对新目标分三组测试：

| 目标 | 结果 | 说明 |
| --- | --- | --- |
| (-205.15, 23, -311.01) | ✅ 到达（距目标 0.36 格） | 爬第一架 4 格梯子 y=19→23，翻上平台后沿 y=23 走廊西行 |
| (-319.05, 22, -366.57) | ✅ 到达（距目标 0.56 格） | 40-50 秒走完 115 格，途中爬第二架梯子 y=19→22→23，无重规划 |
| (-416.73, 28, -369.46) | ⚠️ 分段推进到 -331 | 首段 28 万节点/约 2 分钟返回部分路径即开始行进；自动续段（Segment 1/20）后推进 -270→-292→-306→爬第二架梯子→-331；第三架梯子处服务器拒绝爬升（bot 被钉在 y=22.20，服务器端判定"在梯子旁边"），客户端缓存与服务器对梯子格/朝向的判定不一致，待协议层确认 |

**爬梯修复的验证细节**（3.16）：

1. 修复前：机器人在梯子格内 y=20.20 卡死（头顶梯子被当实心方块）、或在梯子中心无法产生爬升速度、或爬到顶后滑回循环；
2. 修复后：第一架梯子日志 `y 19.00 → 20.06 → 22.55 → 23.00`（连续爬升 + 翻顶），第二架梯子 `y 19.12 → 21.59 → 22.18 → 23.00` 同样完整；
3. 居中爬升生效：机器人进入梯子格前先对准格中心（x/z 偏差收敛到 0.01~0.1 格），不再从边缘滑脱。

**联机同步修复的验证细节**（3.17）：修复前机器人 y 被钉在 19.35/19.60（服务器回显 Teleport 清零速度）；修复后爬升速度保持 `vel=(0, 0.118, 0)` 连续上升，不再被拉回。

**取消机制验证**（3.18）：在 -331 平台角落卡住时，机器人 5 次重规划后输出 `Movement cancelled: cannot escape the current position after repeated retries`，CPU 不再被无上限搜索占用。

**路上规划验证**（3.19）：(-416, 28) 目标首段搜索 28 万节点（约 2 分钟）即返回部分路径，机器人立即开走（旧实现需 5 分钟一次性搜索且可能失败）；到达部分路径终点后输出 `Segment 1/20 reached; re-planning toward final goal...` 并自动续段。服务器回弹检测在 -202 走廊"推墙-回弹"循环中约 1.5 秒触发重规划，避免无限推墙。

## 5 结论与展望

本文通过**规划层 3D 邻域建模与搜索加速**、**执行层环境感知**、**联机场景的服务器信任**三类改进，系统性地解决了 MCC 机器人在人工建筑与大型迷宫地形上的寻路失效问题。关键经验有四：

1. 方块网格搜索的"邻居生成"必须与物理执行能力一致（可落地的过渡），且启发式必须可采纳——平方距离启发式的高估是大型迷宫搜索爆炸的根源；
2. 执行层必须持续探测环境（落地、边缘、居中）而非盲信路径，但**缓存探测只能用于安全模式**：客户端区块缓存与服务器不一致时，`-f` 应信任服务器物理并以重规划兜底；
3. 联机联调时，朝向这类"看起来无害"的状态会被服务器位置包周期性覆盖，路径执行必须显式接管所有权，否则任何方向修正都会被逐 tick 抵消。
4. 特殊方块（梯子）不能只修寻路器：**碰撞形状、爬升物理、输入生成三层必须一致**——梯子在碰撞层要可穿过、在物理层要"有输入即爬升"、在执行层要先居中再贴墙、到顶后要跳离；任何一层按"实心墙"处理都会让爬梯静默失败。

展望：① 将安全坠落深度与 `UnsafeFallDepth` 统一为可配置参数；② 为 `SimplifyPath` 增加 3D 直线平滑（当前仅同平面）；③ 支持 Minecraft 楼梯方块（stairs/slab）的半格形状建模；④ 在 `-f` 模式下引入"服务器区块查询"回退，使缓存缺失时也能精确判断悬崖；⑤ 为"未知区块搜索"引入预算上限与分段导航，避免 240+ 格远目标在未加载地形上展开百万级节点；⑥ 将 Backrooms 迷宫（含多段爬梯）纳入 MCC 官方集成测试场景。

---

## 附录 A：全部修改位置清单（圈出）

### A.1 `MinecraftClient/Mapping/Movement.cs`（1356 行）

| # | 位置 | 函数 | 改动性质 |
| --- | --- | --- | --- |
| M1 | L70-108 | `GetAvailableMoves` | 修改：斜向安全 + 斜上跨步生成 |
| M2 | L110-133 | `IsDiagonal` / `DiagonalCardinalA` / `DiagonalCardinalB` | 新增辅助 |
| M3 | L1252-1270 | `CanStepUp` | 新增 |
| M4 | L210 / L241 | `CalculatePath` | 修改：接入 `SimplifyPath`、双向 A* 与异步入口 |
| M5 | L740-783 | `CanTravelStraight` | 新增 |
| M6 | L785-822 | `SimplifyPath` | 新增 |
| M7 | L821-841 | `IsWalkableColumn` | 新增：3 格支撑规则 |
| M8 | L1164-1184 | `IsSafe` | 修改：目标脚方块非实心 |
| M9 | L1157-1169 | `CanMove`（Up 分支） | 修改：仅攀爬/游泳 |
| M10 | L1278-1298 | `PlayerFitsHere` | 修改：null 安全 |
| M11 | L22-27 | `MoveCostScale` / `UnsafeFallDepth` 常量 | 新增 |
| M12 | L343-463 | `CalculatePathBidirectional` | 新增：双向 A* |
| M13 | L471-505 | `GetReverseNeighbors` | 新增：反向邻居 + 支撑镜像 |
| M14 | L505-535 | `MoveCost` / `Heuristic` | 重写：真实距离启发式 |
| M15 | L538-583 | `GetJumpPointNeighbors` | 新增：JPS 邻居 + 虚空拦截 |
| M16 | L586-660 | `Jump` / `JumpRecursive` | 新增：跳点搜索 |
| M17 | L664-704 | `CanWalkHorizontally` / `IsLoaded` / `IsOpenCell` / `HasForcedNeighbor` | 新增 |
| M18 | L848-868 | `HasSupportWithin` | 新增：5 格支撑 / 未知区块放行 |
| M19 | L242 / L298 / L380 / L465 | `CalculatePath` / `CalculatePathBidirectional` | 修改：节点预算 + 部分路径（bestProgress）返回 |
| M20 | L701-713 | `HasBlockData` | 新增：Y section 级数据存在性检查 |

### A.2 `MinecraftClient/McClient.cs`（5660 行）

| # | 位置 | 函数/字段 | 改动性质 |
| --- | --- | --- | --- |
| C1 | L90-100 | 字段与常量（`movementGoal`、`replanCount`、`wallUnstickTicks`、`movementDebugTicks`、阈值） | 新增 |
| C2 | L1797-1830 | `MoveTo` | 修改：保存目标/参数/重置状态 + 线性距离初始化 + 路径诊断 |
| C3 | L3603-3737 | `UpdatePathfindingInput` | 修改：异步任务接管、推进、拐角恢复、卡住检测、遥测 |
| C4 | L3739-3749 | `ResetPathProgress` | 新增 |
| C5 | L3751-3800 | `ReplanMovement` | 新增：线性距离净进展判定 + 遥测 |
| C6 | L3808-3827 | `ReachedWaypoint` | 重写：落地判定 |
| C7 | L3835-3910 | `SetInputToward`（居中校正 + 墙碰撞定点解卡） | 新增逻辑 |
| C8 | L3925-3945 | `SetInputToward`（自动跳跃） | 新增逻辑 |
| C9 | L3950-3983 | `SetInputToward`（边缘探测） | 修改：`-f` 跳过，安全模式 3 格 |
| C10 | L4001-4015 | `IsCliffSide` | 新增：3 格支撑 |
| C11 | L706-738 | 主循环 yaw/pitch 同步 | 修改：路径活跃时忽略 `_yaw/_pitch` 钉扎 |
| C12 | L3916-3925 | `SetInputToward` 尾部 | 修改：无条件清除 `_yaw/_pitch`（路径接管朝向） |
| C13 | L4043-4053 | `CancelMovement` | 修改：清理目标与后台搜索 |
| C14 | L3850-3935 | `SetInputToward`（爬梯模式） | 新增：居中进梯、贴墙爬升、梯子顶跳离 |
| C15 | L3730-3740 | `UpdatePathfindingInput`（stuck 检测） | 修改：`OnClimbable` 时不计卡住 |
| C16 | L4182-4197 | `UpdateLocation(Location, float, float)` | 修改：路径活跃时忽略服务器小差距回显 |
| C17 | L1821 / L3802 | `MoveTo` / `ReplanMovement` | 修改：搜索超时 120s → 300s |
| C18 | L3621-3630 | 异步任务完成分支 | 修改：不再无条件重置 `replanCount` |
| C19 | L1838-1863 | `StartPathSearch` | 新增：统一异步搜索入口（预算参数化） |
| C20 | L3695-3715 | 分段续段（完成分支） | 新增：未达最终目标自动续段，续段预算翻倍 |
| C21 | L3770-3790 | stuck 检测（服务器回弹） | 新增：位置回退 1.5s 触发重规划 |
| C22 | L3795-3830 | stuck 检测（区块等待）+ `IsAheadChunkLoading` | 新增：section 缺失等待 ≤60s |
| C23 | L3850-3950 | 爬梯模式 | 修改：居中阈值 0.08 + 自适应 facing 交替 |
| C24 | L110 | `WaypointVerticalTolerance` | 修改：0.6 → 0.8 |

### A.3 `MinecraftClient.Tests/MovementTests.cs`（217 行，新增）

10 个测试用例，行号：L45、L59、L75、L89、L103、L126、L144、L165、L178、L200。

### A.4 `MinecraftClient/Physics/PlayerPhysics.cs`（610 行）

| # | 位置 | 函数/字段 | 改动性质 |
| --- | --- | --- | --- |
| P1 | L140-158 | `HandleJumping` | 修改：`OnClimbable` 时允许跳跃 |
| P2 | L206-220 | `TravelInAir`（爬升 bump） | 修改：有移动输入即爬升 |
| P3 | L226-235 | `TravelInAir`（重力分支） | 修改：未知地面跳过重力 |
| P4 | L~371 | `Move()` 的 `OnGround` 修正 | 修改：未知地面视为站立 |
| P5 | L~597-615 | `HasUnknownGround` | 新增 |

### A.5 `MinecraftClient/Physics/CollisionDetector.cs`（210 行）

| # | 位置 | 函数 | 改动性质 |
| --- | --- | --- | --- |
| D1 | L153 | `CollectBlockColliders` | 修改：可攀爬方块（梯子/藤蔓）不产生碰撞盒 |

### A.6 提交状态

全部改动已提交至 `master`（基线 `7b100a31` 之上）：

| commit | 内容 | 对应章节 |
| --- | --- | --- |
| `0ea9f64d` | 路径平滑 + 拐角路点恢复 | 3.1（部分） |
| `842e1a0e` | JPS + 双向 A* + 异步搜索 + 执行层全套 | 3.1-3.9、3.10-3.12 |
| `849c75bf` | `-f` 允许未加载区块 + 居中调参 + 卡点遥测 | 3.13（部分） |
| `7d4e3e72` | 朝向接管 + 定点解卡 + `-f` 服务器信任 + 虚空拦截 + 计数修复 | 3.13-3.15 |
| `5327af6f` | 路上规划（部分路径 + 分段续段）+ 服务器回弹检测 + section 级未知感知 + 自适应爬梯朝向 | 3.16-3.19 |

全部改进已提交至 `master`（累计 5 个 commit），工作区仅剩本文档。

## 附录 B：提交记录与完整代码差异

### B.1 变更规模

相对基线 `7b100a31` 的累计变更：

```text
 MinecraftClient/Mapping/Movement.cs | 754 +++++++++++++++++++++++++--
 MinecraftClient/McClient.cs         | 667 ++++++++++++++++++-
 MinecraftClient/Physics/PlayerPhysics.cs |  45 ++++++-
 MinecraftClient/Physics/CollisionDetector.cs |   6 +
 MinecraftClient.Tests/MovementTests.cs | 217 ++++++++++
 MinecraftClient/Commands/Move.cs       |   7 +-
 6 files changed, 1633 insertions(+), 63 deletions(-)
```

### B.2 第一轮 unified diff（3.1-3.9，相对基线 `7b100a31`）

以下为第一代改进的完整 diff；第二代改动（JPS/双向 A*/异步/`-f` 信任/朝向接管/解卡）的代码片段已逐节列于正文 3.10-3.15，完整内容以 `git show 842e1a0e`、`git show 849c75bf`、`git show 7d4e3e72` 为准。

```diff
diff --git a/MinecraftClient/Mapping/Movement.cs b/MinecraftClient/Mapping/Movement.cs
@@ -1,5 +1,6 @@
 ﻿using System;
 using System.Collections.Generic;
+using System.Linq;
 using System.Threading;
 using System.Threading.Tasks;

@@ -61,9 +62,23 @@ namespace MinecraftClient.Mapping
                 foreach (Direction dir in Enum.GetValues(typeof(Direction)))
                 {
                     Location dest = Move(location, dir);
-                    if (CanMove(world, location, dir) && (allowUnsafe || IsSafe(world, dest)))
+                    bool diagonalSafe = !IsDiagonal(dir)
+                        || (IsSafe(world, Move(location, DiagonalCardinalA(dir)))
+                            && IsSafe(world, Move(location, DiagonalCardinalB(dir))));
+                    if (CanMove(world, location, dir) && (allowUnsafe || (IsSafe(world, dest) && diagonalSafe)))
                         availableMoves.Add(dest);
                 }
+
+                // Step-up: jump onto a 1-high solid block in an adjacent cell.
+                foreach (Direction dir in new[] { Direction.East, Direction.West, Direction.North, Direction.South })
+                {
+                    Location stepUp = Move(Move(location, dir), Direction.Up);
+                    if (CanStepUp(world, location, dir) && (allowUnsafe || IsSafe(world, stepUp)))
+                        availableMoves.Add(stepUp);
+                }
             }
             else
             {
@@ -77,6 +92,36 @@ namespace MinecraftClient.Mapping
             return availableMoves;
         }

+        private static bool IsDiagonal(Direction direction) { ... }
+        private static Direction DiagonalCardinalA(Direction direction) { ... }
+        private static Direction DiagonalCardinalB(Direction direction) { ... }
+
         /// <summary>
         /// Decompose a single move from a block to another into several steps
         /// </summary>
@@ -219,7 +264,7 @@ namespace MinecraftClient.Mapping
                 if ((current.Location == goalLower && maxOffset <= 0) ||
                     (maxOffset > 0 && current.HScore >= minOffset && current.HScore <= maxOffset))
-                    return ReconstructPath(cameFrom, current.Location, start, goal);
+                    return SimplifyPath(world, ReconstructPath(cameFrom, current.Location, start, goal));

                 // Discover neighbored blocks
                 foreach (Location neighbor in GetAvailableMoves(world, current.Location, allowUnsafe))
@@ -249,11 +294,124 @@ namespace MinecraftClient.Mapping
             if (current is not null && openSet.MinHScoreNode is not null &&
                 (maxOffset == int.MaxValue || openSet.MinHScoreNode.HScore <= maxOffset))
-                return ReconstructPath(cameFrom, openSet.MinHScoreNode.Location, start, goal);
+                return SimplifyPath(world, ReconstructPath(cameFrom, openSet.MinHScoreNode.Location, start, goal));

             return null;
         }

+        public static bool CanTravelStraight(...) { ... }   // 全文见 3.1
+        public static Queue<Location> SimplifyPath(...) { ... }
+        private static bool IsWalkableColumn(...) { ... }
+
         /// <summary>
         /// Helper function for CalculatePath(). Backtrack from goal to start ...
         /// </summary>
@@ -558,8 +716,11 @@ namespace MinecraftClient.Mapping
         private static bool IsSafe(World world, Location location)
         {
             return
+                !world.GetBlock(location).Type.IsSolid()
+
                 //No block that can harm the player
-                !world.GetBlock(location).Type.CanHarmPlayers()
+                && !world.GetBlock(location).Type.CanHarmPlayers()
                 && !world.GetBlock(Move(location, Direction.Up)).Type.CanHarmPlayers()
                 && !world.GetBlock(Move(location, Direction.Down)).Type.CanHarmPlayers()
@@ -592,15 +753,18 @@ namespace MinecraftClient.Mapping
                 case Direction.Down:
                     return IsClimbing(world, Move(location, Direction.Down)) || !IsOnGround(world, location);
                 case Direction.Up:
-                    bool nextTwoBlocks =
-                        !world.GetBlock(Move(Move(location, Direction.Up), Direction.Up)).Type.IsSolid();
-
-                    if (IsClimbing(world, location))
-                        return IsClimbing(world, Move(location, Direction.Up)) || nextTwoBlocks;
-
-                    return (IsOnGround(world, location) || IsSwimming(world, location)) && nextTwoBlocks;
+                    if (IsClimbing(world, location))
+                        return IsClimbing(world, Move(location, Direction.Up))
+                            || (!world.GetBlock(Move(location, Direction.Up)).Type.IsSolid()
+                                && !world.GetBlock(Move(Move(location, Direction.Up), Direction.Up)).Type.IsSolid());
+                    return IsSwimming(world, location)
+                        && !world.GetBlock(Move(location, Direction.Up)).Type.IsSolid()
+                        && !world.GetBlock(Move(Move(location, Direction.Up), Direction.Up)).Type.IsSolid();
@@ -628,10 +792,35 @@ namespace MinecraftClient.Mapping
         }

+        private static bool CanStepUp(World world, Location location, Direction direction) { ... }
+
         /// <summary>
         /// Evaluates if a player fits in this location
         /// </summary>
@@ -645,9 +834,12 @@ namespace MinecraftClient.Mapping
             // Handle slabs
-            if (!isNotSolid && world.GetBlock(Move(location, Direction.Up))
-                    .IsTopSlab(McClient.Instance!.GetProtocolVersion()))
+            int? protocolVersion = McClient.Instance?.GetProtocolVersion();
+            if (!isNotSolid && protocolVersion is not null &&
+                world.GetBlock(Move(location, Direction.Up)).IsTopSlab(protocolVersion.Value))
+            {
                 isNotSolid = true;
+            }

             return canClimb || isNotSolid;
         }
@@ -713,4 +905,4 @@ namespace MinecraftClient.Mapping
             return true;
         }
     }
-}
\ No newline at end of file
+}
diff --git a/MinecraftClient/McClient.cs b/MinecraftClient/McClient.cs
@@ -83,6 +83,18 @@ namespace MinecraftClient
         private readonly MovementInput physicsInput = new();
         private bool physicsInitialized = false;
         private Location? pathTarget;
+        private Location? movementGoal;
+        private bool movementAllowUnsafe;
+        private int movementMaxOffset;
+        private int movementMinOffset;
+        private double lastWaypointDistanceSqr = double.MaxValue;
+        private double lastReplanGoalDistanceSqr = double.MaxValue;
+        private int pathStuckTicks;
+        private int replanCount;
+        private const int PathStuckThresholdTicks = 60;
+        private const int MaxReplansWithoutProgress = 5;
+        private const double PathProgressEpsilonSqr = 0.0025;
+        private const double WaypointVerticalTolerance = 0.6;
         public enum MovementType { Sneak, Walk, Sprint }
@@ -1783,8 +1795,21 @@ namespace MinecraftClient
                 else
                 {
+                    movementGoal = goal;
+                    movementAllowUnsafe = allowUnsafe;
+                    movementMaxOffset = maxOffset;
+                    movementMinOffset = minOffset;
+                    replanCount = 0;
+                    lastReplanGoalDistanceSqr = goal.DistanceSquared(location);
                     pathTarget = null;
                     path = Movement.CalculatePath(...);
+                    if (path is not null)
+                        ConsoleIO.WriteLineFormatted($"§e[MCC] Path ({path.Count}): ...");
+                    lastWaypointDistanceSqr = double.MaxValue;
+                    pathStuckTicks = 0;
                     return path is not null;
                 }
@@ -3558,6 +3583,7 @@ namespace MinecraftClient
                 if (path is not null && path.Count > 0)
                 {
                     pathTarget = path.Dequeue();
+                    ResetPathProgress(pathTarget.Value);
                     if (Config.Main.Advanced.MoveHeadWhileWalking)
@@ -3565,6 +3591,26 @@ namespace MinecraftClient
                 {
                     pathTarget = null;
                     path = null;
+                    movementGoal = null;
+                }
+            }
+            else if (pathTarget is not null && path is not null && path.Count > 0)
+            {
+                // Corner recovery: skip current waypoint when straight to next is clear
+                Location next = path.Peek();
+                if (curDx * curDx + curDz * curDz < 0.5625 &&
+                    Math.Abs(pathTarget.Value.Y - location.Y) < WaypointVerticalTolerance &&
+                    playerPhysics.OnGround &&
+                    Movement.CanTravelStraight(world, location, next))
+                {
+                    pathTarget = path.Dequeue();
+                    ResetPathProgress(pathTarget.Value);
                 }
             }
@@ -3572,16 +3618,107 @@ namespace MinecraftClient
             if (pathTarget is not null)
             {
+                // Stuck detection + replan（全文见 3.5）
                 SetInputToward(pathTarget.Value);
             }
         }

+        private void ResetPathProgress(Location target) { ... }
+        private void ReplanMovement() { ... }
+
         /// <summary>
         /// Check if the player has approximately reached a waypoint.
         /// </summary>
@@ -3589,7 +3726,23 @@ namespace MinecraftClient
         {
             double dx = target.X - location.X;
             double dz = target.Z - location.Z;
-            return dx * dx + dz * dz < 0.25;
+            if (dx * dx + dz * dz >= 0.49) return false;
+            double dy = target.Y - location.Y;
+            if (Math.Abs(dy) >= WaypointVerticalTolerance) return false;
+            return playerPhysics.OnGround
+                || playerPhysics.InWater
+                || playerPhysics.OnClimbable
+                || playerPhysics.VerticalCollisionBelow;
         }
@@ -3605,17 +3758,84 @@ namespace MinecraftClient
             if (distSqr < 0.01) return;

-            float targetYaw = (float)(-Math.Atan2(dx, dz) / Math.PI * 180.0);
+            // 居中校正（全文见 3.8）
+            float targetYaw = (float)(-Math.Atan2(dx + steerX, dz + steerZ) / Math.PI * 180.0);
             if (targetYaw < 0) targetYaw += 360;
             playerPhysics.Yaw = targetYaw;
             playerYaw = targetYaw;

             physicsInput.Forward = true;

-            if (dy > 0.5 && playerPhysics.OnGround)
-                physicsInput.Jump = true;
+            // 自动跳跃（全文见 3.6）
+            // 边缘探测（全文见 3.7）

             // Map MovementSpeed setting: 1=sneak, 2-4=walk, 5=sprint
@@ -3624,6 +3844,20 @@ namespace MinecraftClient
                 physicsInput.Sneak = true;
         }

+        private bool IsCliffSide(World world, Location location, Direction direction) { ... }
+
         /// <summary>
         /// Check if the client is currently processing a Movement.
         /// </summary>
@@ -3650,6 +3884,7 @@ namespace MinecraftClient
         {
             bool success = ClientIsMoving();
             path = null;
+            movementGoal = null;
             return success;
         }
```

> 注：附录 B 为结构化的 diff 摘要（完整函数体见第 3 节各小节；`{ ... }` 处为省略的完整实现）。如需逐字节完整 diff，可在仓库中执行：
>
> ```bash
> cd Minecraft-Console-Client && git diff 7b100a31 -- MinecraftClient/Mapping/Movement.cs MinecraftClient/McClient.cs
> ```

## 参考文献

1. MCCTeam. Minecraft-Console-Client（AGENTS.md、docs/guide/ai-assisted-development.md）[EB/OL]. https://github.com/MCCTeam/Minecraft-Console-Client
2. Hart P E, Nilsson N J, Raphael B. A formal basis for the heuristic determination of minimum cost paths[J]. IEEE Transactions on Systems Science and Cybernetics, 1968, 4(2): 100-107.
3. Mojang. Minecraft Wiki: 玩家移动与跳跃（跳跃高度与距离）[EB/OL]. https://minecraft.wiki/
