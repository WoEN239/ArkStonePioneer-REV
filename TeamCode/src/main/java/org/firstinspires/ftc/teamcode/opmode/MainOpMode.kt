package org.firstinspires.ftc.teamcode.opmode

import kotlin.random.Random

class MainOpMode : BaseOpMode() {

    override fun main() {
        execute(
            arrayOf(
                Runnable { robot.movement.rotateAbsolute(45.0) },
                Runnable { robot.startButton.awaitPress() },
                Runnable { robot.movement.move(150.0) },
                Runnable { robot.movement.rotateAbsolute(180.0) },
                Runnable { robot.movement.approachWall() },
                Runnable { robot.movement.rotateAbsolute(-90.0) },
                Runnable { robot.movement.approachWall() },
                Runnable { robot.movement.rotateAbsolute(.0) }
            )
        )
        while (opModeIsActive()) {
            execute(arrayOf(
                Runnable { robot.movement.approachWall() },
                Runnable {
                    robot.movement.rotate(
                        Random.nextDouble(
                            60.0,
                            150.0
                        ) * (if (Random.nextBoolean()) -1.0 else 1.0)
                    )
                }
            ))
            if (robot.barrier.secondsSinceLastBarrierOpening > 40) {
                execute(
                    if (Random.nextBoolean())
                        arrayOf(
                            Runnable { robot.movement.rotateAbsolute(180.0) },
                            Runnable { robot.movement.approachWall() },
                            Runnable { robot.movement.rotateAbsolute(-90.0) },
                        )
                    else
                        arrayOf(
                            Runnable { robot.movement.rotateAbsolute(-90.0) },
                            Runnable { robot.movement.approachWall() },
                            Runnable { robot.movement.rotateAbsolute(180.0) },
                        )
                )
                execute(
                    arrayOf(
                        Runnable { robot.movement.approachWall() },
                        Runnable { robot.movement.rotateAbsolute(45.0) },
                        Runnable { robot.movement.approachWall() },
                    )
                )
            }
        }
    }
}