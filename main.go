package main

import (
	"image/color"
	"log"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

const (
	screenWidth  = 1200
	screenHeight = 720
)

var (
	emptyImage = ebiten.NewImage(3, 3)
)

func init() {
	emptyImage.Fill(color.White)
}

type Game struct {
	physicsActive bool
}

func (g *Game) Update() error {
	touches := inpututil.AppendJustPressedTouchIDs(nil)
	if inpututil.IsKeyJustPressed(ebiten.KeySpace) ||
		inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) ||
		len(touches) > 0 {

		g.physicsActive = !g.physicsActive
		if g.physicsActive {
			InitPhysics()
		}
	}

	if g.physicsActive {
		RunPhysicsStep()
	}

	return nil
}

func (g *Game) Draw(screen *ebiten.Image) {

	for i := 0; i < GetPhysicsBodiesCount(); i++ {
		body := GetPhysicsBody(i)
		if body == nil {
			continue
		}
		vertexCount := GetPhysicsShapeVerticesCount(i)
		for j := 0; j < vertexCount; j++ {
			// Get physics bodies shape vertices to draw lines
			// Note: GetPhysicsShapeVertex() already calculates rotation transformations
			vertexA := GetPhysicsShapeVertex(body, j)

			jj := 0
			if j+1 < vertexCount {
				jj = j + 1 // Get next vertex or first to close the shape
			}
			vertexB := GetPhysicsShapeVertex(body, jj)

			ebitenutil.DrawLine(screen,
				vertexA.X, vertexA.Y,
				vertexB.X, vertexB.Y,
				color.RGBA{0, 255, 0, 255},
			)
		}
	}

	ebitenutil.DebugPrint(screen, "Press <space> or click to start/stop")
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (int, int) {
	return screenWidth, screenHeight
}

func main() {
	ebiten.SetWindowSize(screenWidth, screenHeight)
	ebiten.SetWindowTitle("Physac golang demo")

	// Create floor rectangle physics body
	floor := CreatePhysicsBodyRectangle(Vector2{screenWidth / 2, screenHeight * 0.975},
		screenWidth*1.3, screenHeight/5, 10)
	SetPhysicsBodyRotation(floor, (math.Pi/180)*3)
	floor.Enabled = false // Disable body state to convert it to static (no dynamics, but collisions)
	floor.Restitution = 1

	floor2 := CreatePhysicsBodyRectangle(Vector2{screenWidth / 2, screenHeight * 0.975},
		screenWidth*1.3, screenHeight/5, 10)
	SetPhysicsBodyRotation(floor2, (math.Pi/180)*-3)
	floor2.Enabled = false // Disable body state to convert it to static (no dynamics, but collisions)
	floor2.Restitution = 1

	circle := CreatePhysicsBodyCircle(Vector2{screenWidth * 0.10, screenHeight * 0.1},
		screenWidth*0.02, 10)
	circle.Restitution = 0.5

	circle2 := CreatePhysicsBodyCircle(Vector2{screenWidth * 0.90, screenHeight * 0.2},
		screenWidth*0.03, 10)
	circle2.Restitution = 0.3

	circle3 := CreatePhysicsBodyCircle(Vector2{screenWidth * 0.50, screenHeight * 0.05},
		screenWidth*0.015, 10)
	circle3.Restitution = 1

	SetPhysicsGravity(0, 9.81/10)

	if err := ebiten.RunGame(&Game{}); err != nil {
		log.Fatal(err)
	}
}
