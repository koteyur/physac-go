/**********************************************************************************************
*
*   Physac-go - 2D Physics library for videogames. It's based on original Physac library v1.1
*   written in pure C: https://github.com/victorfisac/Physac
*
*   DESCRIPTION:
*
*   Physac is a small 2D physics library written in go. A physics step contains the following phases:
*   get collision information, apply dynamics, collision solving and position correction.
*   It uses a very simple struct for physic bodies with a position vector to be used in any 2D rendering API.
*
*   LICENSE: zlib/libpng
*
*   Copyright (c) 2016-2020 Victor Fisac (github: @victorfisac)
*
*   This software is provided "as-is", without any express or implied warranty. In no event
*   will the authors be held liable for any damages arising from the use of this software.
*
*   Permission is granted to anyone to use this software for any purpose, including commercial
*   applications, and to alter it and redistribute it freely, subject to the following restrictions:
*
*     1. The origin of this software must not be misrepresented; you must not claim that you
*     wrote the original software. If you use this software in a product, an acknowledgment
*     in the product documentation would be appreciated but is not required.
*
*     2. Altered source versions must be plainly marked as such, and must not be misrepresented
*     as being the original software.
*
*     3. This notice may not be removed or altered from any source distribution.
*
**********************************************************************************************/

package main

import (
	"math"
	"math/rand"
	"time"
)

const (
	PHYSAC_MAX_BODIES      = 64
	PHYSAC_MAX_MANIFOLDS   = 4096
	PHYSAC_MAX_VERTICES    = 24
	PHYSAC_CIRCLE_VERTICES = 24

	PHYSAC_COLLISION_ITERATIONS   = 100
	PHYSAC_PENETRATION_ALLOWANCE  = 0.05
	PHYSAC_PENETRATION_CORRECTION = 0.4

	PHYSAC_PI      = 3.141592653589793
	PHYSAC_DEG2RAD = PHYSAC_PI / 180

	PHYSAC_FLT_MAX = 3.402823466e+38
	PHYSAC_EPSILON = 0.000001
	PHYSAC_K       = 1.0 / 3.0
)

type Vector2 struct {
	X float64
	Y float64
}

type PhysicsShapeType int

const (
	PHYSICS_CIRCLE = PhysicsShapeType(iota)
	PHYSICS_POLYGON
)

type PhysicsBody *PhysicsBodyData

// Mat2 type is used for polygon shape rotation matrix
type Mat2 struct {
	M00 float64
	M01 float64
	M10 float64
	M11 float64
}

type PolygonData struct {
	VertexCount uint                         // Current used vertex and normals count
	Positions   [PHYSAC_MAX_VERTICES]Vector2 // Polygon vertex positions vectors
	Normals     [PHYSAC_MAX_VERTICES]Vector2 // Polygon vertex normals vectors
}

type PhysicsShape struct {
	Type       PhysicsShapeType // Physics shape type (circle or polygon)
	Body       PhysicsBody      // Shape physics body reference
	Radius     float64          // Circle shape radius (used for circle shapes)
	Transform  Mat2             // Vertices transform matrix 2x2
	VertexData PolygonData      // Polygon shape vertices position and normals data (just used for polygon shapes)
}

type PhysicsBodyData struct {
	ID              uint         // Reference unique identifier
	Enabled         bool         // Enabled dynamics state (collisions are calculated anyway)
	Position        Vector2      // Physics body shape pivot
	Velocity        Vector2      // Current linear velocity applied to position
	Force           Vector2      // Current linear force (reset to 0 every step)
	AngularVelocity float64      // Current angular velocity applied to orient
	Torque          float64      // Current angular force (reset to 0 every step)
	Orient          float64      // Rotation in radians
	Inertia         float64      // Moment of inertia
	InverseInertia  float64      // Inverse value of inertia
	Mass            float64      // Physics body mass
	InverseMass     float64      // Inverse value of mass
	StaticFriction  float64      // Friction when the body has not movement (0 to 1)
	DynamicFriction float64      // Friction when the body has movement (0 to 1)
	Restitution     float64      // Restitution coefficient of the body (0 to 1)
	UseGravity      bool         // Apply gravity force to dynamics
	IsGrounded      bool         // Physics grounded on other body state
	FreezeOrient    bool         // Physics rotation constraint
	Shape           PhysicsShape // Physics body shape information (type, radius, vertices, normals)
}

type PhysicsManifoldData struct {
	ID              uint        // Reference unique identifier
	BodyA           PhysicsBody // Manifold first physics body reference
	BodyB           PhysicsBody // Manifold second physics body reference
	Penetration     float64     // Depth of penetration from collision
	Normal          Vector2     // Normal direction vector from 'a' to 'b'
	Contacts        [2]Vector2  // Points of contact during collision
	ContactsCount   uint        // Current collision number of contacts
	Restitution     float64     // Mixed restitution during collision
	DynamicFriction float64     // Mixed dynamic friction during collision
	StaticFriction  float64     // Mixed static friction during collision
}

type PhysicsManifold *PhysicsManifoldData

var (
	// Offset time for MONOTONIC clock
	baseTime = time.Now()

	// Start time in milliseconds
	startTime float64 = 0.0

	// Delta time used for physics steps, in milliseconds
	deltaTime float64 = 1.0 / 60.0 / 10.0 * 1000

	// Current time in milliseconds
	currentTime float64 = 0.0

	// Hi-res clock frequency
	frequency uint64 = 0

	// Physics time step delta time accumulator
	accumulator float64 = 0.0

	// Total physics steps processed
	stepsCount uint = 0

	// Physics world gravity force
	gravityForce Vector2 = Vector2{0.0, 9.81}

	// Physics bodies pointers array
	bodies [64]PhysicsBody

	// Physics world current bodies counter
	physicsBodiesCount uint = 0

	// Physics bodies pointers array
	contacts [4096]PhysicsManifold

	// Physics world current manifolds counter
	physicsManifoldsCount uint = 0
)

// InitPhysics initializes physics values, pointers and creates physics loop thread
func InitPhysics() {
	initTimer()
	accumulator = 0.0
}

// SetPhysicsGravity sets physics global gravity force
func SetPhysicsGravity(x float64, y float64) {
	gravityForce.X = x
	gravityForce.Y = y
}

// CreatePhysicsBodyCircle creates a new circle physics body with generic parameters
func CreatePhysicsBodyCircle(pos Vector2, radius float64, density float64) PhysicsBody {
	var newID int = findAvailableBodyIndex()
	if newID < 0 {
		return nil
	}

	// Initialize new body with generic values
	newBody := &PhysicsBodyData{
		ID:              uint(newID),
		Enabled:         true,
		Position:        pos,
		Velocity:        Vector2{},
		Force:           Vector2{},
		AngularVelocity: 0.0,
		Torque:          0.0,
		Orient:          0.0,
		Shape: PhysicsShape{
			Type:       PHYSICS_CIRCLE,
			Radius:     radius,
			Transform:  mat2Radians(0.0),
			VertexData: PolygonData{},
		},
		StaticFriction:  0.4,
		DynamicFriction: 0.2,
		Restitution:     0.0,
		UseGravity:      true,
		IsGrounded:      false,
		FreezeOrient:    false,
	}

	newBody.Shape.Body = newBody

	newBody.Mass = PHYSAC_PI * radius * radius * density
	newBody.InverseMass = safeDiv(1.0, newBody.Mass)
	newBody.Inertia = newBody.Mass * radius * radius
	newBody.InverseInertia = safeDiv(1.0, newBody.Inertia)

	// Add new body to bodies pointers array and update bodies count
	bodies[physicsBodiesCount] = newBody
	physicsBodiesCount++
	return newBody
}

// CreatePhysicsBodyRectangle creates a new rectangle physics body with generic parameters
func CreatePhysicsBodyRectangle(pos Vector2, width float64, height float64, density float64) PhysicsBody {
	var newID int = findAvailableBodyIndex()
	if newID < 0 {
		return nil
	}

	// Initialize new body with generic values
	newBody := &PhysicsBodyData{
		ID:              uint(newID),
		Enabled:         true,
		Position:        pos,
		Velocity:        Vector2{},
		Force:           Vector2{},
		AngularVelocity: 0.0,
		Torque:          0.0,
		Orient:          0.0,
		Shape: PhysicsShape{
			Type:       PHYSICS_POLYGON,
			Transform:  mat2Radians(0.0),
			VertexData: createRectanglePolygon(pos, Vector2{width, height}),
		},
		StaticFriction:  0.4,
		DynamicFriction: 0.2,
		Restitution:     0.0,
		UseGravity:      true,
		IsGrounded:      false,
		FreezeOrient:    false,
	}

	// Calculate centroid and moment of inertia
	newBody.Shape.Body = newBody
	var center Vector2
	area := 0.0
	inertia := 0.0

	for i := 0; i < int(newBody.Shape.VertexData.VertexCount); i++ {
		// Triangle vertices, third vertex implied as (0, 0)
		nextIndex := getNextIndex(i, int(newBody.Shape.VertexData.VertexCount))
		p1 := newBody.Shape.VertexData.Positions[i]
		p2 := newBody.Shape.VertexData.Positions[nextIndex]

		D := mathCrossVector2(p1, p2)
		triangleArea := D / 2

		area += triangleArea

		// Use area to weight the centroid average, not just vertex position
		center.X += triangleArea * PHYSAC_K * (p1.X + p2.X)
		center.Y += triangleArea * PHYSAC_K * (p1.Y + p2.Y)

		var intx2 float64 = p1.X*p1.X + p2.X*p1.X + p2.X*p2.X
		var inty2 float64 = p1.Y*p1.Y + p2.Y*p1.Y + p2.Y*p2.Y
		inertia += (0.25 * PHYSAC_K * D) * (intx2 + inty2)
	}

	center.X *= 1.0 / area
	center.Y *= 1.0 / area

	// Translate vertices to centroid (make the centroid (0, 0) for the polygon in model space)
	// Note: this is not really necessary
	for i := 0; i < int(newBody.Shape.VertexData.VertexCount); i++ {
		newBody.Shape.VertexData.Positions[i].X -= center.X
		newBody.Shape.VertexData.Positions[i].Y -= center.Y
	}

	newBody.Mass = density * area
	newBody.InverseMass = safeDiv(1.0, newBody.Mass)
	newBody.Inertia = density * inertia
	newBody.InverseInertia = safeDiv(1.0, newBody.Inertia)

	// Add new body to bodies pointers array and update bodies count
	bodies[physicsBodiesCount] = newBody
	physicsBodiesCount++
	return newBody
}

// CreatePhysicsBodyPolygon creates a new polygon physics body with generic parameters
func CreatePhysicsBodyPolygon(pos Vector2, radius float64, sides int, density float64) PhysicsBody {
	var newID int = findAvailableBodyIndex()
	if newID < 0 {
		return nil
	}

	// Initialize new body with generic values
	newBody := &PhysicsBodyData{
		ID:              uint(newID),
		Enabled:         true,
		Position:        pos,
		Velocity:        Vector2{},
		Force:           Vector2{},
		AngularVelocity: 0.0,
		Torque:          0.0,
		Orient:          0.0,
		Shape: PhysicsShape{
			Type:       PHYSICS_POLYGON,
			Transform:  mat2Radians(0.0),
			VertexData: createRandomPolygon(radius, sides),
		},
		StaticFriction:  0.4,
		DynamicFriction: 0.2,
		Restitution:     0.0,
		UseGravity:      true,
		IsGrounded:      false,
		FreezeOrient:    false,
	}

	newBody.Shape.Body = newBody

	// Calculate centroid and moment of inertia
	var center Vector2
	area := 0.0
	inertia := 0.0

	for i := 0; i < int(newBody.Shape.VertexData.VertexCount); i++ {
		// Triangle vertices, third vertex implied as (0, 0)
		nextIndex := getNextIndex(i, int(newBody.Shape.VertexData.VertexCount))
		position1 := newBody.Shape.VertexData.Positions[i]
		position2 := newBody.Shape.VertexData.Positions[nextIndex]

		cross := mathCrossVector2(position1, position2)
		triangleArea := cross / 2

		area += triangleArea

		// Use area to weight the centroid average, not just vertex position
		center.X += triangleArea * PHYSAC_K * (position1.X + position2.X)
		center.Y += triangleArea * PHYSAC_K * (position1.Y + position2.Y)

		intx2 := position1.X*position1.X + position2.X*position1.X + position2.X*position2.X
		inty2 := position1.Y*position1.Y + position2.Y*position1.Y + position2.Y*position2.Y
		inertia += (0.25 * PHYSAC_K * cross) * (intx2 + inty2)
	}

	center.X *= 1.0 / area
	center.Y *= 1.0 / area

	// Translate vertices to centroid (make the centroid (0, 0) for the polygon in model space)
	// Note: this is not really necessary
	for i := 0; i < int(newBody.Shape.VertexData.VertexCount); i++ {
		newBody.Shape.VertexData.Positions[i].X -= center.X
		newBody.Shape.VertexData.Positions[i].Y -= center.Y
	}

	newBody.Mass = density * area
	newBody.InverseMass = safeDiv(1.0, newBody.Mass)
	newBody.Inertia = density * inertia
	newBody.InverseInertia = safeDiv(1.0, newBody.Inertia)

	// Add new body to bodies pointers array and update bodies count
	bodies[physicsBodiesCount] = newBody
	physicsBodiesCount++
	return newBody
}

// PhysicsAddForce adds a force to a physics body
func PhysicsAddForce(body PhysicsBody, force Vector2) {
	if body != nil {
		body.Force = vector2Add(body.Force, force)
	}
}

// PhysicsAddTorque adds an angular force to a physics body
func PhysicsAddTorque(body PhysicsBody, amount float64) {
	if body != nil {
		body.Torque += amount
	}
}

// PhysicsShatter shatters a polygon shape physics body to little physics bodies with explosion force
func PhysicsShatter(body PhysicsBody, position Vector2, force float64) {
	if body == nil || body.Shape.Type != PHYSICS_POLYGON {
		return
	}

	vertexData := body.Shape.VertexData
	collision := false

	for i := 0; i < int(vertexData.VertexCount); i++ {
		nextIndex := getNextIndex(i, int(vertexData.VertexCount))
		pA := body.Position
		pB := mat2MultiplyVector2(body.Shape.Transform,
			vector2Add(body.Position, vertexData.Positions[i]))
		pC := mat2MultiplyVector2(body.Shape.Transform,
			vector2Add(body.Position, vertexData.Positions[nextIndex]))

		// Check collision between each triangle
		alpha := ((pB.Y-pC.Y)*(position.X-pC.X) + (pC.X-pB.X)*(position.Y-pC.Y)) /
			((pB.Y-pC.Y)*(pA.X-pC.X) + (pC.X-pB.X)*(pA.Y-pC.Y))
		beta := ((pC.Y-pA.Y)*(position.X-pC.X) + (pA.X-pC.X)*(position.Y-pC.Y)) /
			((pB.Y-pC.Y)*(pA.X-pC.X) + (pC.X-pB.X)*(pA.Y-pC.Y))
		gamma := 1.0 - alpha - beta

		if alpha > 0.0 && beta > 0.0 && gamma > 0.0 {
			collision = true
			break
		}
	}

	if !collision {
		return
	}

	count := int(vertexData.VertexCount)
	bodyPos := body.Position
	trans := body.Shape.Transform

	vertices := make([]Vector2, count)
	for i := 0; i < count; i++ {
		vertices[i] = vertexData.Positions[i]
	}

	// Destroy shattered physics body
	DestroyPhysicsBody(body)
	for i := 0; i < count; i++ {
		nextIndex := getNextIndex(i, count)
		center := triangleBarycenter(vertices[i], vertices[nextIndex], Vector2{})
		center = vector2Add(bodyPos, center)
		offset := vector2Subtract(center, bodyPos)

		var newBody PhysicsBody = CreatePhysicsBodyPolygon(center, 10, 3, 10)
		var newData PolygonData = PolygonData{}
		newData.VertexCount = 3
		newData.Positions[0] = vector2Subtract(vertices[i], offset)
		newData.Positions[1] = vector2Subtract(vertices[nextIndex], offset)
		newData.Positions[2] = vector2Subtract(position, center)

		// Separate vertices to avoid unnecessary physics collisions
		newData.Positions[0].X *= 0.95
		newData.Positions[0].Y *= 0.95
		newData.Positions[1].X *= 0.95
		newData.Positions[1].Y *= 0.95
		newData.Positions[2].X *= 0.95
		newData.Positions[2].Y *= 0.95

		// Calculate polygon faces normals
		for j := 0; j < int(newData.VertexCount); j++ {
			nextVertex := getNextIndex(j, int(newData.VertexCount))
			face := vector2Subtract(newData.Positions[nextVertex], newData.Positions[j])

			newData.Normals[j] = Vector2{face.Y, -face.X}
			mathNormalize(&newData.Normals[j])
		}

		// Apply computed vertex data to new physics body shape
		newBody.Shape.VertexData = newData
		newBody.Shape.Transform = trans

		// Calculate centroid and moment of inertia
		center = Vector2{}
		var area float64 = 0.0
		var inertia float64 = 0.0

		for j := 0; j < int(newBody.Shape.VertexData.VertexCount); j++ {
			// Triangle vertices, third vertex implied as (0, 0)
			nextVertex := getNextIndex(j, int(newBody.Shape.VertexData.VertexCount))
			p1 := newBody.Shape.VertexData.Positions[j]
			p2 := newBody.Shape.VertexData.Positions[nextVertex]

			D := mathCrossVector2(p1, p2)
			triangleArea := D / 2

			area += triangleArea

			// Use area to weight the centroid average, not just vertex position
			center.X += triangleArea * PHYSAC_K * (p1.X + p2.X)
			center.Y += triangleArea * PHYSAC_K * (p1.Y + p2.Y)

			intx2 := p1.X*p1.X + p2.X*p1.X + p2.X*p2.X
			inty2 := p1.Y*p1.Y + p2.Y*p1.Y + p2.Y*p2.Y
			inertia += (0.25*PHYSAC_K*D)*intx2 + inty2
		}

		center.X *= 1.0 / area
		center.Y *= 1.0 / area

		newBody.Mass = area
		newBody.InverseMass = safeDiv(1.0, newBody.Mass)
		newBody.Inertia = inertia
		newBody.InverseInertia = safeDiv(1.0, newBody.Inertia)

		// Calculate explosion force direction
		pointA := newBody.Position
		pointB := vector2Subtract(newData.Positions[1], newData.Positions[0])
		pointB.X /= 2.0
		pointB.Y /= 2.0
		forceDirection := vector2Subtract(
			vector2Add(pointA, vector2Add(newData.Positions[0], pointB)),
			newBody.Position,
		)
		mathNormalize(&forceDirection)
		forceDirection.X *= force
		forceDirection.Y *= force

		// Apply force to new physics body
		PhysicsAddForce(newBody, forceDirection)
	}
}

// GetPhysicsBodiesCount returns the current amount of created physics bodies
func GetPhysicsBodiesCount() int {
	return int(physicsBodiesCount)
}

// GetPhysicsBody returns a physics body of the bodies pool at a specific index
func GetPhysicsBody(index int) PhysicsBody {
	// if index < int(physicsBodiesCount) {
	// 	if bodies[index] == nil {
	// 	}
	// }
	return bodies[index]
}

// GetPhysicsShapeType returns the physics body shape type (PHYSICS_CIRCLE or PHYSICS_POLYGON)
func GetPhysicsShapeType(index int) int {
	var result int = -1
	if index < int(physicsBodiesCount) {
		if bodies[index] != nil {
			result = int(bodies[index].Shape.Type)
		}
	}
	return result
}

// GetPhysicsShapeVerticesCount returns the amount of vertices of a physics body shape
func GetPhysicsShapeVerticesCount(index int) int {
	var result int = 0
	if index < int(physicsBodiesCount) {
		if bodies[index] != nil {
			switch bodies[index].Shape.Type {
			case PHYSICS_CIRCLE:
				result = PHYSAC_CIRCLE_VERTICES
			case PHYSICS_POLYGON:
				result = int(bodies[index].Shape.VertexData.VertexCount)
			default:
			}
		}
	}
	return result
}

// GetPhysicsShapeVertex returns transformed position of a body shape (body position + vertex transformed position)
func GetPhysicsShapeVertex(body PhysicsBody, vertex int) Vector2 {
	var position Vector2
	if body == nil {
		return position
	}

	switch body.Shape.Type {
	case PHYSICS_CIRCLE:
		angle := 360.0 / PHYSAC_CIRCLE_VERTICES * float64(vertex) * (PHYSAC_DEG2RAD)
		position.X = body.Position.X + math.Cos(angle)*body.Shape.Radius
		position.Y = body.Position.Y + math.Sin(angle)*body.Shape.Radius
	case PHYSICS_POLYGON:
		position = vector2Add(
			body.Position,
			mat2MultiplyVector2(body.Shape.Transform, body.Shape.VertexData.Positions[vertex]),
		)
	}
	return position
}

// SetPhysicsBodyRotation sets physics body shape transform based on radians parameter
func SetPhysicsBodyRotation(body PhysicsBody, radians float64) {
	if body == nil {
		return
	}
	body.Orient = radians
	if body.Shape.Type == PHYSICS_POLYGON {
		body.Shape.Transform = mat2Radians(radians)
	}
}

// DestroyPhysicsBody unitializes and destroy a physics body
func DestroyPhysicsBody(body PhysicsBody) {
	if body == nil {
		return
	}

	id := body.ID
	index := -1
	for i := 0; i < int(physicsBodiesCount); i++ {
		if bodies[i].ID == id {
			index = i
			break
		}
	}
	if index == -1 {
		return
	}

	bodies[index] = nil

	// Reorder physics bodies pointers array and its catched index
	for i := uint(index); i < physicsBodiesCount; i++ {
		if (i + 1) < physicsBodiesCount {
			bodies[i] = bodies[i+1]
		}
	}

	// Update physics bodies count
	physicsBodiesCount--
}

// ClosePhysics unitializes physics pointers and closes physics loop thread
func ClosePhysics() {
	// Unitialize physics manifolds dynamic memory allocations
	for i := int(physicsManifoldsCount - 1); i >= 0; i-- {
		destroyPhysicsManifold(contacts[i])
	}

	// Unitialize physics bodies dynamic memory allocations
	for i := int(physicsBodiesCount - 1); i >= 0; i-- {
		DestroyPhysicsBody(bodies[i])
	}
}

// findAvailableBodyIndex finds a valid index for a new physics body initialization
func findAvailableBodyIndex() int {
	index := -1
	for i := 0; i < PHYSAC_MAX_BODIES; i++ {
		currentID := i

		// Check if current id already exist in other physics body
		for k := 0; k < int(physicsBodiesCount); k++ {
			if bodies[k].ID == uint(currentID) {
				currentID++
				break
			}
		}

		// If it is not used, use it as new physics body id
		if currentID == i {
			index = i
			break
		}
	}
	return index
}

// createRandomPolygon creates a random polygon shape with max vertex distance from polygon pivot
func createRandomPolygon(radius float64, sides int) PolygonData {
	var data PolygonData = PolygonData{}
	data.VertexCount = uint(sides)

	// Calculate polygon vertices positions
	for i := 0; i < int(data.VertexCount); i++ {
		data.Positions[i].X = math.Cos(360.0/float64(sides)*float64(i)*PHYSAC_DEG2RAD) * radius
		data.Positions[i].Y = math.Sin(360.0/float64(sides)*float64(i)*PHYSAC_DEG2RAD) * radius
	}

	// Calculate polygon faces normals
	for i := 0; i < int(data.VertexCount); i++ {
		nextIndex := getNextIndex(i, sides)
		face := vector2Subtract(data.Positions[nextIndex], data.Positions[i])

		data.Normals[i] = Vector2{face.Y, -face.X}
		mathNormalize(&data.Normals[i])
	}

	return data
}

// createRectanglePolygon creates a rectangle polygon shape based on a min and max positions
func createRectanglePolygon(pos Vector2, size Vector2) PolygonData {
	var data PolygonData = PolygonData{}
	data.VertexCount = 4

	// Calculate polygon vertices positions
	data.Positions[0] = Vector2{pos.X + size.X/2, pos.Y - size.Y/2}
	data.Positions[1] = Vector2{pos.X + size.X/2, pos.Y + size.Y/2}
	data.Positions[2] = Vector2{pos.X - size.X/2, pos.Y + size.Y/2}
	data.Positions[3] = Vector2{pos.X - size.X/2, pos.Y - size.Y/2}

	// Calculate polygon faces normals
	for i := 0; i < int(data.VertexCount); i++ {
		nextIndex := getNextIndex(i, int(data.VertexCount))
		face := vector2Subtract(data.Positions[nextIndex], data.Positions[i])

		data.Normals[i] = Vector2{face.Y, -face.X}
		mathNormalize(&data.Normals[i])
	}

	return data
}

// physicsStep physics steps calculations (dynamics, collisions and position corrections)
func physicsStep() {
	// Update current steps count
	stepsCount++

	// Clear previous generated collisions information
	for i := int(physicsManifoldsCount - 1); i >= 0; i-- {
		if manifold := contacts[i]; manifold != nil {
			destroyPhysicsManifold(manifold)
		}
	}

	// Reset physics bodies grounded state
	for i := 0; i < int(physicsBodiesCount); i++ {
		bodies[i].IsGrounded = false
	}

	// Generate new collision information
	for i := 0; i < int(physicsBodiesCount); i++ {
		bodyA := bodies[i]
		if bodyA == nil {
			continue
		}

		for j := i + 1; j < int(physicsBodiesCount); j++ {
			var bodyB PhysicsBody = bodies[j]
			if bodyB == nil || bodyA.InverseMass == 0 && bodyB.InverseMass == 0 {
				continue
			}

			manifold := createPhysicsManifold(bodyA, bodyB)
			solvePhysicsManifold(manifold)

			if manifold.ContactsCount > 0 {
				// Create a new manifold with same information as previously solved manifold and add it to the manifolds pool last slot
				newManifold := createPhysicsManifold(bodyA, bodyB)
				newManifold.Penetration = manifold.Penetration
				newManifold.Normal = manifold.Normal
				newManifold.Contacts[0] = manifold.Contacts[0]
				newManifold.Contacts[1] = manifold.Contacts[1]
				newManifold.ContactsCount = manifold.ContactsCount
				newManifold.Restitution = manifold.Restitution
				newManifold.DynamicFriction = manifold.DynamicFriction
				newManifold.StaticFriction = manifold.StaticFriction
			}
		}
	}

	// Integrate forces to physics bodies
	for i := 0; i < int(physicsBodiesCount); i++ {
		if body := bodies[i]; body != nil {
			integratePhysicsForces(body)
		}
	}

	// Initialize physics manifolds to solve collisions
	for i := 0; i < int(physicsManifoldsCount); i++ {
		if manifold := contacts[i]; manifold != nil {
			initializePhysicsManifolds(manifold)
		}
	}

	// Integrate physics collisions impulses to solve collisions
	for i := 0; i < PHYSAC_COLLISION_ITERATIONS; i++ {
		for j := 0; j < int(physicsManifoldsCount); j++ {
			if manifold := contacts[i]; manifold != nil {
				integratePhysicsImpulses(manifold)
			}
		}
	}

	// Integrate velocity to physics bodies
	for i := 0; i < int(physicsBodiesCount); i++ {
		if body := bodies[i]; body != nil {
			integratePhysicsVelocity(body)
		}
	}

	// Correct physics bodies positions based on manifolds collision information
	for i := 0; i < int(physicsManifoldsCount); i++ {
		if manifold := contacts[i]; manifold != nil {
			correctPhysicsPositions(manifold)
		}
	}

	// Clear physics bodies forces
	for i := 0; i < int(physicsBodiesCount); i++ {
		if body := bodies[i]; body != nil {
			body.Force = Vector2{}
			body.Torque = 0.0
		}
	}
}

// RunPhysicsStep runs physics step, to be used if PHYSICS_NO_THREADS is set in your main loop
func RunPhysicsStep() {
	// Calculate current time
	currentTime = getCurrentTime()

	// Calculate current delta time
	var delta float64 = currentTime - startTime

	// Store the time elapsed since the last frame began
	accumulator += delta

	// Fixed time stepping loop
	for accumulator >= deltaTime {
		physicsStep()
		accumulator -= deltaTime
	}

	// Record the starting of this frame
	startTime = currentTime
}

// SetPhysicsTimeStep sets physics fixed time step in milliseconds. 1.666666 by default
func SetPhysicsTimeStep(delta float64) {
	deltaTime = delta
}

// findAvailableManifoldIndex finds a valid index for a new manifold initialization
func findAvailableManifoldIndex() int {
	index := -1
	for i := 0; i < PHYSAC_MAX_MANIFOLDS; i++ {
		var currentId int = i

		// Check if current id already exist in other physics body
		for k := 0; k < int(physicsManifoldsCount); k++ {
			if contacts[k].ID == uint(currentId) {
				currentId++
				break
			}
		}

		// If it is not used, use it as new physics body id
		if currentId == i {
			index = i
			break
		}
	}
	return index
}

// createPhysicsManifold creates a new physics manifold to solve collision
func createPhysicsManifold(a PhysicsBody, b PhysicsBody) PhysicsManifold {
	newID := findAvailableManifoldIndex()
	if newID < 0 {
		return nil
	}

	// Initialize new manifold with generic values
	newManifold := &PhysicsManifoldData{
		ID:              uint(newID),
		BodyA:           a,
		BodyB:           b,
		Penetration:     0,
		Normal:          Vector2{},
		Contacts:        [2]Vector2{},
		ContactsCount:   0,
		Restitution:     0.0,
		DynamicFriction: 0.0,
		StaticFriction:  0.0,
	}

	// Add new contact to conctas pointers array and update contacts count
	contacts[physicsManifoldsCount] = newManifold
	physicsManifoldsCount++

	return newManifold
}

// destroyPhysicsManifold unitializes and destroys a physics manifold
func destroyPhysicsManifold(manifold PhysicsManifold) {
	if manifold == nil {
		return
	}

	id := int(manifold.ID)
	index := -1
	for i := 0; i < int(physicsManifoldsCount); i++ {
		if contacts[i].ID == uint(id) {
			index = i
			break
		}
	}
	if index < 0 {
		return
	}

	contacts[index] = nil

	// Reorder physics manifolds pointers array and its catched index
	for i := uint(index); i < physicsManifoldsCount; i++ {
		if (i + 1) < physicsManifoldsCount {
			contacts[i] = contacts[i+1]
		}
	}

	// Update physics manifolds count
	physicsManifoldsCount--
}

// solvePhysicsManifold solves a created physics manifold between two physics bodies
func solvePhysicsManifold(manifold PhysicsManifold) {
	switch manifold.BodyA.Shape.Type {
	case PHYSICS_CIRCLE:
		switch manifold.BodyB.Shape.Type {
		case PHYSICS_CIRCLE:
			solveCircleToCircle(manifold)
		case PHYSICS_POLYGON:
			solveCircleToPolygon(manifold)
		}
	case PHYSICS_POLYGON:
		switch manifold.BodyB.Shape.Type {
		case PHYSICS_CIRCLE:
			solvePolygonToCircle(manifold)
		case PHYSICS_POLYGON:
			solvePolygonToPolygon(manifold)
		}
	}

	// Update physics body grounded state if normal direction is down and grounded state
	// is not set yet in previous manifolds
	if !manifold.BodyB.IsGrounded {
		manifold.BodyB.IsGrounded = manifold.Normal.Y < 0
	}
}

// solveCircleToCircle solves collision between two circle shape physics bodies
func solveCircleToCircle(manifold PhysicsManifold) {
	bodyA, bodyB := manifold.BodyA, manifold.BodyB
	if bodyA == nil || bodyB == nil {
		return
	}

	// Calculate translational vector, which is normal
	var normal Vector2 = vector2Subtract(bodyB.Position, bodyA.Position)

	distSqr := mathLenSqr(normal)
	radius := bodyA.Shape.Radius + bodyB.Shape.Radius

	// Check if circles are not in contact
	if distSqr >= radius*radius {
		manifold.ContactsCount = 0
		return
	}

	var distance float64 = math.Sqrt(distSqr)
	manifold.ContactsCount = 1
	if distance == 0.0 {
		manifold.Penetration = bodyA.Shape.Radius
		manifold.Normal = Vector2{1.0, 0.0}
		manifold.Contacts[0] = bodyA.Position
	} else {
		manifold.Penetration = radius - distance
		// Faster than using MathNormalize() due to sqrt is already performed
		manifold.Normal = Vector2{
			normal.X / distance,
			normal.Y / distance,
		}
		manifold.Contacts[0] = Vector2{
			manifold.Normal.X*bodyA.Shape.Radius + bodyA.Position.X,
			manifold.Normal.Y*bodyA.Shape.Radius + bodyA.Position.Y,
		}
	}

	// Update physics body grounded state if normal direction is down
	if !bodyA.IsGrounded {
		bodyA.IsGrounded = manifold.Normal.Y < 0
	}
}

// solveCircleToPolygon solves collision between a circle to a polygon shape physics bodies
func solveCircleToPolygon(manifold PhysicsManifold) {
	bodyA, bodyB := manifold.BodyA, manifold.BodyB
	if bodyA == nil || bodyB == nil {
		return
	}
	solveDifferentShapes(manifold, bodyA, bodyB)
}

// solvePolygonToCircle solves collision between a polygon to a circle shape physics bodies
func solvePolygonToCircle(manifold PhysicsManifold) {
	bodyA, bodyB := manifold.BodyA, manifold.BodyB
	if bodyA == nil || bodyB == nil {
		return
	}
	solveDifferentShapes(manifold, bodyB, bodyA)
	manifold.Normal.X *= -1.0
	manifold.Normal.Y *= -1.0
}

// solveDifferentShapes solves collision between two different types of shapes
func solveDifferentShapes(manifold PhysicsManifold, bodyA PhysicsBody, bodyB PhysicsBody) {
	manifold.ContactsCount = 0

	// Transform circle center to polygon transform space
	center := mat2MultiplyVector2(
		mat2Transpose(bodyB.Shape.Transform),
		vector2Subtract(bodyA.Position, bodyB.Position),
	)

	// Find edge with minimum penetration
	// It is the same concept as using support points in SolvePolygonToPolygon
	var separation float64 = float64(-PHYSAC_FLT_MAX)
	faceNormal := 0
	vertexData := bodyB.Shape.VertexData

	for i := 0; i < int(vertexData.VertexCount); i++ {
		currentSeparation := mathDot(
			vertexData.Normals[i],
			vector2Subtract(center, vertexData.Positions[i]),
		)

		if currentSeparation > bodyA.Shape.Radius {
			return
		}

		if currentSeparation > separation {
			separation = currentSeparation
			faceNormal = i
		}
	}

	// Grab face's vertices
	nextIndex := getNextIndex(faceNormal, int(vertexData.VertexCount))
	v1 := vertexData.Positions[faceNormal]
	v2 := vertexData.Positions[nextIndex]

	// Check to see if center is within polygon
	if separation < PHYSAC_EPSILON {
		manifold.ContactsCount = 1
		var normal Vector2 = mat2MultiplyVector2(bodyB.Shape.Transform, vertexData.Normals[faceNormal])
		manifold.Normal = Vector2{-normal.X, -normal.Y}
		manifold.Contacts[0] = Vector2{
			manifold.Normal.X*bodyA.Shape.Radius + bodyA.Position.X,
			manifold.Normal.Y*bodyA.Shape.Radius + bodyA.Position.Y,
		}
		manifold.Penetration = bodyA.Shape.Radius
		return
	}

	// Determine which voronoi region of the edge center of circle lies within
	dot1 := mathDot(vector2Subtract(center, v1), vector2Subtract(v2, v1))
	dot2 := mathDot(vector2Subtract(center, v2), vector2Subtract(v1, v2))
	manifold.Penetration = bodyA.Shape.Radius - separation

	switch {
	case dot1 <= 0.0: // Closest to v1
		if distSqr(center, v1) > bodyA.Shape.Radius*bodyA.Shape.Radius {
			return
		}

		manifold.ContactsCount = 1
		var normal Vector2 = vector2Subtract(v1, center)
		normal = mat2MultiplyVector2(bodyB.Shape.Transform, normal)
		mathNormalize(&normal)
		manifold.Normal = normal
		v1 = mat2MultiplyVector2(bodyB.Shape.Transform, v1)
		v1 = vector2Add(v1, bodyB.Position)
		manifold.Contacts[0] = v1

	case dot2 <= 0.0: // Closest to v2
		if distSqr(center, v2) > bodyA.Shape.Radius*bodyA.Shape.Radius {
			return
		}

		manifold.ContactsCount = 1
		var normal Vector2 = vector2Subtract(v2, center)
		v2 = mat2MultiplyVector2(bodyB.Shape.Transform, v2)
		v2 = vector2Add(v2, bodyB.Position)
		manifold.Contacts[0] = v2
		normal = mat2MultiplyVector2(bodyB.Shape.Transform, normal)
		mathNormalize(&normal)
		manifold.Normal = normal

	default: // Closest to face
		var normal Vector2 = vertexData.Normals[faceNormal]

		if mathDot(vector2Subtract(center, v1), normal) > bodyA.Shape.Radius {
			return
		}

		normal = mat2MultiplyVector2(bodyB.Shape.Transform, normal)
		manifold.Normal = Vector2{-normal.X, -normal.Y}
		manifold.Contacts[0] = Vector2{
			manifold.Normal.X*bodyA.Shape.Radius + bodyA.Position.X,
			manifold.Normal.Y*bodyA.Shape.Radius + bodyA.Position.Y,
		}
		manifold.ContactsCount = 1
	}
}

// solvePolygonToPolygon solves collision between two polygons shape physics bodies
func solvePolygonToPolygon(manifold PhysicsManifold) {
	bodyA, bodyB := manifold.BodyA, manifold.BodyB
	if bodyA == nil || bodyB == nil {
		return
	}

	shapeA, shapeB := bodyA.Shape, bodyB.Shape
	manifold.ContactsCount = 0

	// Check for separating axis with A shape's face planes
	faceA := 0
	penetrationA := findAxisLeastPenetration(&faceA, shapeA, shapeB)

	if penetrationA >= 0.0 {
		return
	}

	// Check for separating axis with B shape's face planes
	faceB := 0
	penetrationB := findAxisLeastPenetration(&faceB, shapeB, shapeA)

	if penetrationB >= 0.0 {
		return
	}

	referenceIndex := 0
	flip := false // Always point from A shape to B shape

	var refPoly PhysicsShape // Reference
	var incPoly PhysicsShape // Incident

	// Determine which shape contains reference face
	if biasGreaterThan(penetrationA, penetrationB) {
		refPoly = shapeA
		incPoly = shapeB
		referenceIndex = faceA
	} else {
		refPoly = shapeB
		incPoly = shapeA
		referenceIndex = faceB
		flip = bool(true)
	}

	// World space incident face
	var incidentFace [2]Vector2
	findIncidentFace(&incidentFace[0], &incidentFace[1], refPoly, incPoly, referenceIndex)

	// Setup reference face vertices
	refData := refPoly.VertexData
	v1 := refData.Positions[referenceIndex]
	referenceIndex = getNextIndex(referenceIndex, int(refData.VertexCount))
	v2 := refData.Positions[referenceIndex]

	// Transform vertices to world space
	v1 = mat2MultiplyVector2(refPoly.Transform, v1)
	v1 = vector2Add(v1, refPoly.Body.Position)
	v2 = mat2MultiplyVector2(refPoly.Transform, v2)
	v2 = vector2Add(v2, refPoly.Body.Position)

	// Calculate reference face side normal in world space
	sidePlaneNormal := vector2Subtract(v2, v1)
	mathNormalize(&sidePlaneNormal)

	// Orthogonalize
	refFaceNormal := Vector2{sidePlaneNormal.Y, -sidePlaneNormal.X}
	refC := mathDot(refFaceNormal, v1)
	negSide := mathDot(sidePlaneNormal, v1) * float64(-1)
	posSide := mathDot(sidePlaneNormal, v2)

	// Clip incident face to reference face side planes (due to floating point error, possible to not have required points
	if clip(Vector2{-sidePlaneNormal.X, -sidePlaneNormal.Y}, negSide, &incidentFace[0], &incidentFace[1]) < 2 {
		return
	}

	if clip(sidePlaneNormal, posSide, &incidentFace[0], &incidentFace[1]) < 2 {
		return
	}

	// Flip normal if required
	if flip {
		manifold.Normal = Vector2{-refFaceNormal.X, -refFaceNormal.Y}
	} else {
		manifold.Normal = refFaceNormal
	}

	// Keep points behind reference face
	var currentPoint int = 0 // Clipped points behind reference face
	var separation float64 = mathDot(refFaceNormal, incidentFace[0]) - refC

	if separation <= 0.0 {
		manifold.Contacts[currentPoint] = incidentFace[0]
		manifold.Penetration = -separation
		currentPoint++
	} else {
		manifold.Penetration = 0.0
	}

	separation = mathDot(refFaceNormal, incidentFace[1]) - refC

	if separation <= 0.0 {
		manifold.Contacts[currentPoint] = incidentFace[1]
		manifold.Penetration += -separation
		currentPoint++

		// Calculate total penetration average
		manifold.Penetration /= float64(currentPoint)
	}

	manifold.ContactsCount = uint(currentPoint)
}

// integratePhysicsForces integrates physics forces into velocity
func integratePhysicsForces(body PhysicsBody) {
	if body == nil || body.InverseMass == 0.0 || !body.Enabled {
		return
	}

	body.Velocity.X += body.Force.X * body.InverseMass * (deltaTime / 2.0)
	body.Velocity.Y += body.Force.Y * body.InverseMass * (deltaTime / 2.0)

	if body.UseGravity {
		body.Velocity.X += gravityForce.X * (deltaTime / 1000 / 2.0)
		body.Velocity.Y += gravityForce.Y * (deltaTime / 1000 / 2.0)
	}

	if !body.FreezeOrient {
		body.AngularVelocity += body.Torque * body.InverseInertia * (deltaTime / 2.0)
	}
}

// initializePhysicsManifolds initializes physics manifolds to solve collisions
func initializePhysicsManifolds(manifold PhysicsManifold) {
	bodyA, bodyB := manifold.BodyA, manifold.BodyB

	if bodyA == nil || bodyB == nil {
		return
	}

	// // Calculate average restitution, static and dynamic friction
	manifold.Restitution = math.Sqrt(bodyA.Restitution * bodyB.Restitution)
	manifold.StaticFriction = math.Sqrt(bodyA.StaticFriction * bodyB.StaticFriction)
	manifold.DynamicFriction = math.Sqrt(bodyA.DynamicFriction * bodyB.DynamicFriction)

	for i := 0; i < int(manifold.ContactsCount); i++ {
		// Caculate radius from center of mass to contact
		radiusA := vector2Subtract(manifold.Contacts[i], bodyA.Position)
		radiusB := vector2Subtract(manifold.Contacts[i], bodyB.Position)

		crossA := mathCross(bodyA.AngularVelocity, radiusA)
		crossB := mathCross(bodyB.AngularVelocity, radiusB)
		radiusV := Vector2{
			bodyB.Velocity.X + crossB.X - bodyA.Velocity.X - crossA.X,
			bodyB.Velocity.Y + crossB.Y - bodyA.Velocity.Y - crossA.Y,
		}

		// Determine if we should perform a resting collision or not;
		// The idea is if the only thing moving this object is gravity, then the collision should be
		// performed without any restitution
		rad := Vector2{gravityForce.X * deltaTime / 1000, gravityForce.Y * deltaTime / 1000}
		if mathLenSqr(radiusV) < (mathLenSqr(rad) + PHYSAC_EPSILON) {
			manifold.Restitution = 0
		}
	}
}

// integratePhysicsImpulses integrates physics collisions impulses to solve collisions
func integratePhysicsImpulses(manifold PhysicsManifold) {
	bodyA, bodyB := manifold.BodyA, manifold.BodyB

	if bodyA == nil || bodyB == nil {
		return
	}

	// Early out and positional correct if both objects have infinite mass
	if math.Abs(bodyA.InverseMass+bodyB.InverseMass) <= PHYSAC_EPSILON {
		bodyA.Velocity = Vector2{}
		bodyB.Velocity = Vector2{}
		return
	}

	for i := 0; i < int(manifold.ContactsCount); i++ {
		// Calculate radius from center of mass to contact
		radiusA := vector2Subtract(manifold.Contacts[i], bodyA.Position)
		radiusB := vector2Subtract(manifold.Contacts[i], bodyB.Position)

		// Calculate relative velocity
		radiusV := Vector2{
			bodyB.Velocity.X + mathCross(bodyB.AngularVelocity, radiusB).X -
				bodyA.Velocity.X - mathCross(bodyA.AngularVelocity, radiusA).X,
			bodyB.Velocity.Y + mathCross(bodyB.AngularVelocity, radiusB).Y -
				bodyA.Velocity.Y - mathCross(bodyA.AngularVelocity, radiusA).Y,
		}

		// Relative velocity along the normal
		contactVelocity := mathDot(radiusV, manifold.Normal)

		// Do not resolve if velocities are separating
		if contactVelocity > 0.0 {
			return
		}

		raCrossN := mathCrossVector2(radiusA, manifold.Normal)
		rbCrossN := mathCrossVector2(radiusB, manifold.Normal)

		inverseMassSum := bodyA.InverseMass + bodyB.InverseMass +
			(raCrossN*raCrossN)*bodyA.InverseInertia + (rbCrossN*rbCrossN)*bodyB.InverseInertia

		// Calculate impulse scalar value
		impulse := -(manifold.Restitution + 1.0) * contactVelocity
		impulse /= inverseMassSum
		impulse /= float64(manifold.ContactsCount)

		// Apply impulse to each physics body
		impulseV := Vector2{manifold.Normal.X * impulse, manifold.Normal.Y * impulse}

		if bodyA.Enabled {
			bodyA.Velocity.X += bodyA.InverseMass * (-impulseV.X)
			bodyA.Velocity.Y += bodyA.InverseMass * (-impulseV.Y)

			if !bodyA.FreezeOrient {
				bodyA.AngularVelocity += bodyA.InverseInertia *
					mathCrossVector2(radiusA, Vector2{-impulseV.X, -impulseV.Y})
			}
		}

		if bodyB.Enabled {
			bodyB.Velocity.X += bodyB.InverseMass * impulseV.X
			bodyB.Velocity.Y += bodyB.InverseMass * impulseV.Y

			if !bodyB.FreezeOrient {
				bodyB.AngularVelocity += bodyB.InverseInertia * mathCrossVector2(radiusB, impulseV)
			}
		}

		// Apply friction impulse to each physics body
		radiusV.X = 0 +
			bodyB.Velocity.X + mathCross(bodyB.AngularVelocity, radiusB).X -
			bodyA.Velocity.X - mathCross(bodyA.AngularVelocity, radiusA).X
		radiusV.Y = 0 +
			bodyB.Velocity.Y + mathCross(bodyB.AngularVelocity, radiusB).Y -
			bodyA.Velocity.Y - mathCross(bodyA.AngularVelocity, radiusA).Y

		tangent := Vector2{
			radiusV.X - manifold.Normal.X*mathDot(radiusV, manifold.Normal),
			radiusV.Y - manifold.Normal.Y*mathDot(radiusV, manifold.Normal),
		}
		mathNormalize(&tangent)

		// Calculate impulse tangent magnitude
		impulseTangent := -mathDot(radiusV, tangent)
		impulseTangent /= inverseMassSum
		impulseTangent /= float64(manifold.ContactsCount)

		absImpulseTangent := math.Abs(impulseTangent)

		// Don't apply tiny friction impulses
		if absImpulseTangent <= PHYSAC_EPSILON {
			return
		}

		// Apply coulumb's law
		var tangentImpulse Vector2
		if absImpulseTangent < impulse*manifold.StaticFriction {
			tangentImpulse = Vector2{tangent.X * impulseTangent, tangent.Y * impulseTangent}
		} else {
			tangentImpulse = Vector2{
				tangent.X * (-impulse) * manifold.DynamicFriction,
				tangent.Y * (-impulse) * manifold.DynamicFriction,
			}
		}

		// Apply friction impulse
		if bodyA.Enabled {
			bodyA.Velocity.X += bodyA.InverseMass * (-tangentImpulse.X)
			bodyA.Velocity.Y += bodyA.InverseMass * (-tangentImpulse.Y)

			if !bodyA.FreezeOrient {
				bodyA.AngularVelocity += bodyA.InverseInertia *
					mathCrossVector2(radiusA, Vector2{-tangentImpulse.X, -tangentImpulse.Y})
			}
		}

		if bodyB.Enabled {
			bodyB.Velocity.X += bodyB.InverseMass * tangentImpulse.X
			bodyB.Velocity.Y += bodyB.InverseMass * tangentImpulse.Y

			if !bodyB.FreezeOrient {
				bodyB.AngularVelocity += bodyB.InverseInertia * mathCrossVector2(radiusB, tangentImpulse)
			}
		}
	}
}

// integratePhysicsVelocity integrates physics velocity into position and forces
func integratePhysicsVelocity(body PhysicsBody) {
	if body == nil || !body.Enabled {
		return
	}

	body.Position.X += body.Velocity.X * deltaTime
	body.Position.Y += body.Velocity.Y * deltaTime

	if !body.FreezeOrient {
		body.Orient += body.AngularVelocity * deltaTime
	}

	mat2Set(&body.Shape.Transform, body.Orient)

	integratePhysicsForces(body)
}

// correctPhysicsPositions corrects physics bodies positions based on manifolds collision information
func correctPhysicsPositions(manifold PhysicsManifold) {
	bodyA, bodyB := manifold.BodyA, manifold.BodyB
	if bodyA == nil || bodyB == nil {
		return
	}

	corrCoeff := math.Max(manifold.Penetration-PHYSAC_PENETRATION_ALLOWANCE, 0.0) /
		(bodyA.InverseMass + bodyB.InverseMass) * PHYSAC_PENETRATION_CORRECTION
	correction := Vector2{corrCoeff * manifold.Normal.X, corrCoeff * manifold.Normal.Y}

	if bodyA.Enabled {
		bodyA.Position.X -= correction.X * bodyA.InverseMass
		bodyA.Position.Y -= correction.Y * bodyA.InverseMass
	}

	if bodyB.Enabled {
		bodyB.Position.X += correction.X * bodyB.InverseMass
		bodyB.Position.Y += correction.Y * bodyB.InverseMass
	}
}

// getSupport returns the extreme point along a direction within a polygon
func getSupport(shape PhysicsShape, dir Vector2) Vector2 {
	var (
		bestProjection float64     = float64(-PHYSAC_FLT_MAX)
		bestVertex     Vector2     = Vector2{}
		data           PolygonData = shape.VertexData
	)
	for i := 0; i < int(data.VertexCount); i++ {
		var (
			vertex     Vector2 = data.Positions[i]
			projection float64 = mathDot(vertex, dir)
		)
		if projection > bestProjection {
			bestVertex = vertex
			bestProjection = projection
		}
	}
	return bestVertex
}

// findAxisLeastPenetration finds polygon shapes axis least penetration
func findAxisLeastPenetration(faceIndex *int, shapeA PhysicsShape, shapeB PhysicsShape) float64 {
	var (
		bestDistance float64     = float64(-PHYSAC_FLT_MAX)
		bestIndex    int         = 0
		dataA        PolygonData = shapeA.VertexData
	)
	for i := 0; i < int(dataA.VertexCount); i++ {
		var (
			normal      Vector2 = dataA.Normals[i]
			transNormal Vector2 = mat2MultiplyVector2(shapeA.Transform, normal)
			buT         Mat2    = mat2Transpose(shapeB.Transform)
		)
		normal = mat2MultiplyVector2(buT, transNormal)
		var support Vector2 = getSupport(shapeB, Vector2{-normal.X, -normal.Y})
		var vertex Vector2 = dataA.Positions[i]
		vertex = mat2MultiplyVector2(shapeA.Transform, vertex)
		vertex = vector2Add(vertex, shapeA.Body.Position)
		vertex = vector2Subtract(vertex, shapeB.Body.Position)
		vertex = mat2MultiplyVector2(buT, vertex)
		var distance float64 = mathDot(normal, vector2Subtract(support, vertex))
		if distance > bestDistance {
			bestDistance = distance
			bestIndex = i
		}
	}
	*faceIndex = bestIndex
	return bestDistance
}

// findIncidentFace finds two polygon shapes incident face
func findIncidentFace(v0 *Vector2, v1 *Vector2, ref PhysicsShape, inc PhysicsShape, index int) {
	var (
		refData         PolygonData = ref.VertexData
		incData         PolygonData = inc.VertexData
		referenceNormal Vector2     = refData.Normals[index]
	)
	// To world space
	referenceNormal = mat2MultiplyVector2(ref.Transform, referenceNormal)
	// To incident's model space
	referenceNormal = mat2MultiplyVector2(mat2Transpose(inc.Transform), referenceNormal)
	var incidentFace int = 0
	var minDot float64 = float64(PHYSAC_FLT_MAX)
	for i := 0; i < int(incData.VertexCount); i++ {
		var dot float64 = mathDot(referenceNormal, incData.Normals[i])
		if dot < minDot {
			minDot = dot
			incidentFace = i
		}
	}
	*v0 = mat2MultiplyVector2(inc.Transform, incData.Positions[incidentFace])
	*v0 = vector2Add(*v0, inc.Body.Position)
	if (incidentFace + 1) < int(incData.VertexCount) {
		incidentFace++
	} else {
		incidentFace = 0
	}
	*v1 = mat2MultiplyVector2(inc.Transform, incData.Positions[incidentFace])
	*v1 = vector2Add(*v1, inc.Body.Position)
}

// clip calculates clipping based on a normal and two faces
func clip(normal Vector2, clip float64, faceA *Vector2, faceB *Vector2) int {
	var (
		sp        = 0
		out       = [2]Vector2{*faceA, *faceB}
		distanceA = mathDot(normal, *faceA) - clip
		distanceB = mathDot(normal, *faceB) - clip
	)

	// If negative (behind plane)
	if distanceA <= 0.0 {
		out[sp] = *faceA
		sp++
	}

	if distanceB <= 0.0 {
		out[sp] = *faceB
		sp++
	}

	// If the points are on different sides of the plane
	if distanceA*distanceB < 0.0 {
		// Push intersection point
		alpha := distanceA / (distanceA - distanceB)
		out[sp] = *faceA
		delta := vector2Subtract(*faceB, *faceA)
		delta.X *= alpha
		delta.Y *= alpha
		out[sp] = vector2Add(out[sp], delta)
		sp++
	}

	// Assign the new converted values
	*faceA = out[0]
	*faceB = out[1]
	return sp
}

// biasGreaterThan checks if values are between bias range
func biasGreaterThan(valueA float64, valueB float64) bool {
	return valueA >= (valueB*0.95 + valueA*0.01)
}

// triangleBarycenter returns the barycenter of a triangle given by 3 points
func triangleBarycenter(v1 Vector2, v2 Vector2, v3 Vector2) Vector2 {
	return Vector2{
		(v1.X + v2.X + v3.X) / 3,
		(v1.Y + v2.Y + v3.Y) / 3,
	}
}

// initTimer initializes hi-resolution MONOTONIC timer
func initTimer() {
	rand.Seed(getTimeCount())
	frequency = 1000000000
	startTime = getCurrentTime() // Get current time
}

// getTimeCount gets hi-res MONOTONIC time measure in nanoseconds
func getTimeCount() int64 {
	return time.Since(baseTime).Nanoseconds()
}

// getCurrentTime gets current time measure in milliseconds
func getCurrentTime() float64 {
	return float64(getTimeCount()) / float64(frequency) * 1000
}

// mathCross returns the cross product of a vector and a value
func mathCross(value float64, vector Vector2) Vector2 {
	return Vector2{-value * vector.Y, value * vector.X}
}

// mathCrossVector2 returns the cross product of two vectors
func mathCrossVector2(v1 Vector2, v2 Vector2) float64 {
	return v1.X*v2.Y - v1.Y*v2.X
}

// mathLenSqr returns the len square root of a vector
func mathLenSqr(vector Vector2) float64 {
	return vector.X*vector.X + vector.Y*vector.Y
}

// mathDot returns the dot product of two vectors
func mathDot(v1 Vector2, v2 Vector2) float64 {
	return v1.X*v2.X + v1.Y*v2.Y
}

// distSqr returns the square root of distance between two vectors
func distSqr(v1 Vector2, v2 Vector2) float64 {
	dir := vector2Subtract(v1, v2)
	return mathDot(dir, dir)
}

// mathNormalize returns the normalized values of a vector
func mathNormalize(vector *Vector2) {
	aux := *vector
	length := math.Sqrt(aux.X*aux.X + aux.Y*aux.Y)
	if length == 0 {
		length = 1.0
	}
	ilength := 1.0 / length
	vector.X *= ilength
	vector.Y *= ilength
}

// vector2Add returns the sum of two given vectors
func vector2Add(v1 Vector2, v2 Vector2) Vector2 {
	return Vector2{v1.X + v2.X, v1.Y + v2.Y}
}

// vector2Subtract returns the subtract of two given vectors
func vector2Subtract(v1 Vector2, v2 Vector2) Vector2 {
	return Vector2{v1.X - v2.X, v1.Y - v2.Y}
}

// mat2Radians creates a matrix 2x2 from a given radians value
func mat2Radians(radians float64) Mat2 {
	cos, sin := math.Cos(radians), math.Sin(radians)
	return Mat2{
		M00: cos,
		M01: -sin,
		M10: sin,
		M11: cos,
	}
}

// mat2Set sets values from radians to a created matrix 2x2
func mat2Set(matrix *Mat2, radians float64) {
	cos, sin := math.Cos(radians), math.Sin(radians)
	matrix.M00 = cos
	matrix.M01 = -sin
	matrix.M10 = sin
	matrix.M11 = cos
}

// mat2Transpose returns the transpose of a given matrix 2x2
func mat2Transpose(matrix Mat2) Mat2 {
	return Mat2{
		M00: matrix.M00,
		M01: matrix.M10,
		M10: matrix.M01,
		M11: matrix.M11,
	}
}

// mat2MultiplyVector2 multiplies a vector by a matrix 2x2
func mat2MultiplyVector2(matrix Mat2, vector Vector2) Vector2 {
	return Vector2{
		matrix.M00*vector.X + matrix.M01*vector.Y,
		matrix.M10*vector.X + matrix.M11*vector.Y,
	}
}

func getNextIndex(i, count int) int {
	if i+1 < count {
		return i + 1
	}
	return 0
}

func safeDiv(a, b float64) float64 {
	if b == 0 {
		return 0
	}
	return a / b
}
