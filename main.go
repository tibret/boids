package main

import (
	"fmt"
	"github.com/gen2brain/raylib-go/raylib"
	"math"
	"math/rand"
)

func main() {
	const screenSize = 1024
	const screenSizeFloat float64 = 1024
	const num_boids = 60
	const speed float64 = 3
	rl.InitWindow(screenSize, screenSize, "boids")
	rl.SetTargetFPS(60)

	target := rl.LoadRenderTexture(screenSize, screenSize)

	var cohesionFactor float32 = 1
	var alignmentFactor float32 = 1
	var avoidanceFactor float32 = 1
	var fov int = 200

	setup := false
	oneFrame := false

	var boids []*Boid = make([]*Boid, num_boids)
	var cameraBoid *Boid

	for i := range num_boids {
		boids[i] = NewBoid(screenSize)
		if i == 0 {
			boids[i].Debug = true
		}
	}

	camera := rl.Camera2D{
		Target:   rl.Vector2{X: screenSize / 2, Y: screenSize / 2},
		Zoom:     1,
		Rotation: 0,
		Offset:   rl.Vector2{X: screenSize / 2, Y: screenSize / 2},
	}

	// invert := rl.LoadShader("shaders/pass.vs", "shaders/invert.fs")
	chromaticAbberation := rl.LoadShader("shaders/pass.vs", "shaders/chromaticabberation.fs")

	for !rl.WindowShouldClose() {
		rl.BeginTextureMode(target)
		rl.BeginDrawing()

		if cameraBoid != nil {
			camera.Target = cameraBoid.Vector2
		}
		camera.Zoom = float32(math.Exp(math.Log(float64(camera.Zoom)) + float64(rl.GetMouseWheelMove()*0.1)))

		rl.BeginMode2D(camera)
		rl.ClearBackground(rl.Black)

		if rl.IsKeyPressed(rl.KeySpace) {
			setup = !setup
		}

		if rl.IsKeyPressed(rl.KeyN) {
			oneFrame = true
		}

		if rl.IsKeyPressed(rl.KeyQ) {
			cohesionFactor += 0.25
		} else if rl.IsKeyPressed(rl.KeyA) {
			cohesionFactor -= 0.25
		}

		if rl.IsKeyPressed(rl.KeyW) {
			alignmentFactor += 0.25
		} else if rl.IsKeyPressed(rl.KeyS) {
			alignmentFactor -= 0.25
		}

		if rl.IsKeyPressed(rl.KeyE) {
			avoidanceFactor += 0.25
		} else if rl.IsKeyPressed(rl.KeyD) {
			avoidanceFactor -= 0.25
		}

		if rl.IsKeyPressed(rl.KeyR) {
			fov += 10
		} else if rl.IsKeyPressed(rl.KeyF) {
			fov -= 10
		}

		userVector := GetUserVector()

		if !setup || oneFrame {
			MoveBoids(&boids, speed, screenSizeFloat, cohesionFactor, alignmentFactor, avoidanceFactor, userVector, fov)
		}

		if rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
			if cameraBoid != nil {
				cameraBoid.Camera = false
			}
			pos := rl.Vector2Subtract(rl.GetMousePosition(), rl.Vector2Subtract(camera.Offset, camera.Target))
			cameraBoid = FindBoid(pos, &boids)
			if cameraBoid != nil {
				cameraBoid.Camera = true
			} else {
				//reset camera
				camera.Target = rl.Vector2{X: screenSize / 2, Y: screenSize / 2}
			}
		}

		if rl.IsMouseButtonPressed(rl.MouseButtonRight) {
			pos := rl.Vector2Subtract(rl.GetMousePosition(), rl.Vector2Subtract(camera.Offset, camera.Target))

			debugBoid := FindBoid(pos, &boids)
			if debugBoid != nil {
				debugBoid.Debug = !debugBoid.Debug
			}
		}

		uiOffset := rl.Vector2Subtract(camera.Offset, camera.Target)
		offsetX, offsetY := int32(uiOffset.X), int32(uiOffset.Y)

		//draw border
		rl.DrawRectangleLines(0, 0, screenSize, screenSize, rl.White)

		DrawBoids(&boids, cohesionFactor, alignmentFactor, avoidanceFactor, userVector, fov)

		oneFrame = false

		rl.EndDrawing()
		rl.EndTextureMode()

		rl.BeginShaderMode(chromaticAbberation)
		rl.DrawTextureRec(target.Texture, rl.NewRectangle(0, 0, float32(target.Texture.Width), -1*float32(target.Texture.Height)), rl.NewVector2(0, 0), rl.White)
		rl.EndShaderMode()

		// draw text on top of shaded texture
		rl.DrawText(fmt.Sprintf("Cohesion: %f", cohesionFactor), 10-offsetX, 10-offsetY, 16, rl.Blue)
		rl.DrawText(fmt.Sprintf("Alignment: %f", alignmentFactor), 10-offsetX, 30-offsetY, 16, rl.Blue)
		rl.DrawText(fmt.Sprintf("Avoidance: %f", avoidanceFactor), 10-offsetX, 50-offsetY, 16, rl.Blue)
		rl.DrawText(fmt.Sprintf("FOV: %d", fov), 10-offsetX, 70-offsetY, 16, rl.Blue)
	}

	rl.CloseWindow()
}

func MoveBoids(boids *[]*Boid, speed float64, screenSize float64, cohesionFactor float32, alignmentFactor float32, avoidanceFactor float32, userVector rl.Vector2, fov int) {
	const rot_speed = 2 * math.Pi / 100

	for _, boid := range *boids {
		neighbors := GatherNeighbors(boid, boids, fov)

		boid.X = float32(math.Mod((float64(boid.X) + speed*math.Cos(boid.Rotation) + screenSize), screenSize))
		boid.Y = float32(math.Mod((float64(boid.Y) + speed*math.Sin(boid.Rotation) + screenSize), screenSize))

		if len(neighbors) == 0 && (userVector.X != 0 || userVector.Y != 0) {
			avg_rot := math.Atan2(float64(userVector.Y), float64(userVector.X))
			boid.Rotation = ClampRot(avg_rot, boid.Rotation, rot_speed)
			continue
		}

		if len(neighbors) == 0 {
			continue
		}

		alignmentVec, cohesionVec, avoidVec := CalculateMovementVecs(boid, &neighbors, cohesionFactor, alignmentFactor, avoidanceFactor)
		totalVec := rl.Vector2{X: avoidVec.X + cohesionVec.X + alignmentVec.X, Y: avoidVec.Y + cohesionVec.Y + alignmentVec.Y}

		if boid.Debug {
			totalVec = rl.Vector2Add(totalVec, userVector)
		}

		avg_rot := math.Atan2(float64(totalVec.Y), float64(totalVec.X))

		//move towards desired rotation by max allowed amount
		boid.Rotation = ClampRot(avg_rot, boid.Rotation, rot_speed)
	}
}

func CalculateMovementVecs(boid *Boid, neighbors *[]*Boid, cohesionFactor float32, alignmentFactor float32, avoidanceFactor float32) (rl.Vector2, rl.Vector2, rl.Vector2) {
	numNeighbors := len(*neighbors)

	//Alignment
	var total_rot float64 = 0
	var tot_x float32 = boid.X
	var tot_y float32 = boid.Y
	var avoidVec rl.Vector2 = rl.NewVector2(0, 0)
	for _, n := range *neighbors {
		tot_x += n.X
		tot_y += n.Y
		total_rot += n.Rotation

		mag := (200 - rl.Vector2Distance(boid.Vector2, n.Vector2))

		angle := math.Atan2(float64(n.Y-boid.Y), float64(n.X-boid.X)) + math.Pi
		// angle := rl.Vector2Angle(boid.Vector2, n.Vector2)
		xMag := float32(float64(mag) * math.Cos(float64(angle)))
		yMag := float32(float64(mag) * math.Sin(float64(angle)))
		avoidVec.X += xMag
		avoidVec.Y += yMag
	}

	//alignment
	alignment_rot := total_rot / float64(numNeighbors)
	alignmentVec := rl.NewVector2(float32(50*math.Cos(alignment_rot))*alignmentFactor, float32(50*math.Sin(alignment_rot))*alignmentFactor)

	//cohesion
	avg_x := tot_x / float32(numNeighbors+1)
	avg_y := tot_y / float32(numNeighbors+1)
	cohesionVec := rl.NewVector2((avg_x-boid.X)*cohesionFactor, (avg_y-boid.Y)*cohesionFactor)

	//avoidance
	avoidVec.X = avoidanceFactor * avoidVec.X / float32(numNeighbors)
	avoidVec.Y = avoidanceFactor * avoidVec.Y / float32(numNeighbors)

	return alignmentVec, cohesionVec, avoidVec
}

func DrawBoids(boids *[]*Boid, cohesionFactor float32, alignmentFactor float32, avoidanceFactor float32, userVector rl.Vector2, fov int) {
	const boid_size float64 = 15
	for _, boid := range *boids {
		tA, tB, tC := BoidPoints(boid)
		color := rl.White
		if boid.Debug {
			color = rl.Blue
			var rotationDeg = boid.Rotation * 180 / math.Pi
			rl.DrawCircleSectorLines(boid.Vector2, 200, float32(rotationDeg-float64(fov/2)), float32(rotationDeg+float64(fov/2)), 1000, rl.Green)
			// rl.DrawText(fmt.Sprintf("Rot: %f", boid.Rotation), int32(boid.X)+10, int32(boid.Y), 20, rl.Blue)

			neighbors := GatherNeighbors(boid, boids, fov)
			if len(neighbors) != 0 {
				alignmentVec, cohesionVec, avoidVec := CalculateMovementVecs(boid, &neighbors, cohesionFactor, alignmentFactor, avoidanceFactor)
				rl.DrawLine(int32(boid.X), int32(boid.Y), int32(boid.X+cohesionVec.X), int32(boid.Y+cohesionVec.Y), rl.Orange)
				rl.DrawLine(int32(boid.X), int32(boid.Y), int32(boid.X+alignmentVec.X), int32(boid.Y+alignmentVec.Y), rl.Yellow)
				rl.DrawLine(int32(boid.X), int32(boid.Y), int32(boid.X+avoidVec.X), int32(boid.Y+avoidVec.Y), rl.Pink)
				rl.DrawLine(int32(boid.X), int32(boid.Y), int32(boid.X+userVector.X), int32(boid.Y+userVector.Y), rl.Blue)
				rl.DrawLine(int32(boid.X), int32(boid.Y), int32(boid.X+avoidVec.X+cohesionVec.X+alignmentVec.X), int32(boid.Y+avoidVec.Y+cohesionVec.Y+alignmentVec.Y), rl.White)
			}
		}
		if boid.Camera {
			color = rl.Green
		}
		rl.DrawTriangle(tC, tB, tA, color)
	}
}

func BoidPoints(boid *Boid) (rl.Vector2, rl.Vector2, rl.Vector2) {
	const boid_size float64 = 15
	tri1 := rl.NewVector2(boid.X+float32(boid_size*math.Cos(boid.Rotation)), boid.Y+float32(boid_size*math.Sin(boid.Rotation)))
	tri2 := rl.NewVector2(boid.X+float32(boid_size*math.Cos(boid.Rotation+(5*math.Pi/6))), boid.Y+float32(boid_size*math.Sin(boid.Rotation+(5*math.Pi/6))))
	tri3 := rl.NewVector2(boid.X+float32(boid_size*math.Cos(boid.Rotation-(5*math.Pi/6))), boid.Y+float32(boid_size*math.Sin(boid.Rotation-(5*math.Pi/6))))

	return tri1, tri2, tri3
}

func FindBoid(pos rl.Vector2, boids *[]*Boid) *Boid {
	for _, boid := range *boids {
		tA, tB, tC := BoidPoints(boid)
		testAB := rl.Vector2CrossProduct(rl.Vector2Subtract(tB, tA), rl.Vector2Subtract(tA, pos))
		testBC := rl.Vector2CrossProduct(rl.Vector2Subtract(tC, tB), rl.Vector2Subtract(tB, pos))
		testCA := rl.Vector2CrossProduct(rl.Vector2Subtract(tA, tC), rl.Vector2Subtract(tC, pos))

		if testAB <= 0 && testBC <= 0 && testCA <= 0 {
			return boid
		}
	}
	return nil
}

func GatherNeighbors(boid *Boid, boids *[]*Boid, fov int) []*Boid {
	const local_distance = 200
	var fovRad float64 = float64(fov) * math.Pi / 180

	var neighbors []*Boid
	for _, b := range *boids {
		if b == boid {
			continue
		}

		dist := rl.Vector2Distance(boid.Vector2, b.Vector2)
		angle := AngleBetween(boid.Vector2, b.Vector2)

		if dist < local_distance {
			diff := math.Mod(angle-boid.Rotation, 2*math.Pi)

			if diff > fovRad/2*-1 && diff < fovRad/2 {
				neighbors = append(neighbors, b)
			}
		}
	}

	return neighbors
}

func AngleBetween(vec1 rl.Vector2, vec2 rl.Vector2) float64 {
	return math.Atan2(float64(vec2.Y-vec1.Y), float64(vec2.X-vec1.X))
}

type Boid struct {
	rl.Vector2
	Rotation float64
	Debug    bool
	Camera   bool
}

func NewBoid(screenSize int) *Boid {
	return &Boid{Vector2: rl.Vector2{X: rand.Float32() * float32(screenSize), Y: rand.Float32() * float32(screenSize)}, Rotation: rand.Float64() * 2 * math.Pi, Debug: false}
}

func ClampRot(desired float64, current float64, rot_speed float64) float64 {
	if math.Abs(desired-current) > rot_speed {
		if desired > current {
			return math.Mod(current+rot_speed, 2*math.Pi)
		} else {
			return math.Mod(current-rot_speed, 2*math.Pi)
		}
	} else {
		return desired
	}
}

func GetUserVector() rl.Vector2 {
	userVector := rl.Vector2{X: 0, Y: 0}
	if rl.IsKeyDown(rl.KeyUp) {
		userVector.Y = userVector.Y - 100
	}
	if rl.IsKeyDown(rl.KeyDown) {
		userVector.Y = userVector.Y + 100
	}
	if rl.IsKeyDown(rl.KeyLeft) {
		userVector.X = userVector.X - 100
	}
	if rl.IsKeyDown(rl.KeyRight) {
		userVector.X = userVector.X + 100
	}

	return userVector
}
