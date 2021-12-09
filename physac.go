package main

import (
	"math"
	"math/rand"
	"time"
)

const PHYSAC_MAX_BODIES = 64
const PHYSAC_MAX_MANIFOLDS = 4096
const PHYSAC_MAX_VERTICES = 24
const PHYSAC_CIRCLE_VERTICES = 24
const PHYSAC_COLLISION_ITERATIONS = 100
const PHYSAC_PENETRATION_ALLOWANCE = 0.05
const PHYSAC_PENETRATION_CORRECTION = 0.4
const PHYSAC_PI = 3.141592653589793
const PHYSAC_FLT_MAX = 3.402823466e+38
const PHYSAC_EPSILON = 1e-06

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
type Mat2 struct {
	M00 float64
	M01 float64
	M10 float64
	M11 float64
}
type PolygonData struct {
	VertexCount uint
	Positions   [24]Vector2
	Normals     [24]Vector2
}
type PhysicsShape struct {
	Type       PhysicsShapeType
	Body       PhysicsBody
	Radius     float64
	Transform  Mat2
	VertexData PolygonData
}
type PhysicsBodyData struct {
	Id              uint
	Enabled         bool
	Position        Vector2
	Velocity        Vector2
	Force           Vector2
	AngularVelocity float64
	Torque          float64
	Orient          float64
	Inertia         float64
	InverseInertia  float64
	Mass            float64
	InverseMass     float64
	StaticFriction  float64
	DynamicFriction float64
	Restitution     float64
	UseGravity      bool
	IsGrounded      bool
	FreezeOrient    bool
	Shape           PhysicsShape
}
type PhysicsManifoldData struct {
	Id              uint
	BodyA           PhysicsBody
	BodyB           PhysicsBody
	Penetration     float64
	Normal          Vector2
	Contacts        [2]Vector2
	ContactsCount   uint
	Restitution     float64
	DynamicFriction float64
	StaticFriction  float64
}
type PhysicsManifold *PhysicsManifoldData

var physicsThreadEnabled bool = bool(false)
var baseTime float64 = 0.0
var startTime float64 = 0.0
var deltaTime float64 = 1.0 / 60.0 / 10.0 * 1000
var currentTime float64 = 0.0
var frequency uint64 = 0
var accumulator float64 = 0.0
var stepsCount uint = 0
var gravityForce Vector2 = Vector2{X: 0.0, Y: 9.81}
var bodies [64]PhysicsBody
var physicsBodiesCount uint = 0
var contacts [4096]PhysicsManifold
var physicsManifoldsCount uint = 0

func InitPhysics() {
	InitTimer()
	accumulator = 0.0
}
func IsPhysicsEnabled() bool {
	return physicsThreadEnabled
}
func SetPhysicsGravity(x float64, y float64) {
	gravityForce.X = x
	gravityForce.Y = y
}
func CreatePhysicsBodyCircle(pos Vector2, radius float64, density float64) PhysicsBody {
	var newBody PhysicsBody = new(PhysicsBodyData)
	var newId int = FindAvailableBodyIndex()
	if newId != -1 {
		newBody.Id = uint(newId)
		newBody.Enabled = bool(true)
		newBody.Position = pos
		newBody.Velocity = Vector2{X: 0.0, Y: 0.0}
		newBody.Force = Vector2{X: 0.0, Y: 0.0}
		newBody.AngularVelocity = 0.0
		newBody.Torque = 0.0
		newBody.Orient = 0.0
		newBody.Shape.Type = PhysicsShapeType(PHYSICS_CIRCLE)
		newBody.Shape.Body = newBody
		newBody.Shape.Radius = radius
		newBody.Shape.Transform = Mat2Radians(0.0)
		newBody.Shape.VertexData = PolygonData{}
		newBody.Mass = float64(PHYSAC_PI * float64(radius) * float64(radius) * float64(density))
		if float64(newBody.Mass) != 0.0 {
			newBody.InverseMass = float64(1.0 / float64(newBody.Mass))
		} else {
			newBody.InverseMass = 0.0
		}
		newBody.Inertia = newBody.Mass * radius * radius
		if float64(newBody.Inertia) != 0.0 {
			newBody.InverseInertia = float64(1.0 / float64(newBody.Inertia))
		} else {
			newBody.InverseInertia = 0.0
		}
		newBody.StaticFriction = 0.4
		newBody.DynamicFriction = 0.2
		newBody.Restitution = 0.0
		newBody.UseGravity = bool(true)
		newBody.IsGrounded = bool(false)
		newBody.FreezeOrient = bool(false)
		bodies[physicsBodiesCount] = newBody
		physicsBodiesCount++
	}
	return newBody
}
func CreatePhysicsBodyRectangle(pos Vector2, width float64, height float64, density float64) PhysicsBody {
	var newBody PhysicsBody = new(PhysicsBodyData)
	var newId int = FindAvailableBodyIndex()
	if newId != -1 {
		newBody.Id = uint(newId)
		newBody.Enabled = bool(true)
		newBody.Position = pos
		newBody.Velocity = Vector2{X: 0.0}
		newBody.Force = Vector2{X: 0.0}
		newBody.AngularVelocity = 0.0
		newBody.Torque = 0.0
		newBody.Orient = 0.0
		newBody.Shape.Type = PhysicsShapeType(PHYSICS_POLYGON)
		newBody.Shape.Body = newBody
		newBody.Shape.Radius = 0.0
		newBody.Shape.Transform = Mat2Radians(0.0)
		newBody.Shape.VertexData = CreateRectanglePolygon(pos, Vector2{X: width, Y: height})
		var center Vector2 = Vector2{X: 0.0, Y: 0.0}
		var area float64 = 0.0
		var inertia float64 = 0.0
		for i := int(0); i < int(newBody.Shape.VertexData.VertexCount); i++ {
			var (
				p1        Vector2 = newBody.Shape.VertexData.Positions[i]
				nextIndex int     = (func() int {
					if (i + 1) < int(newBody.Shape.VertexData.VertexCount) {
						return i + 1
					}
					return 0
				}())
				p2           Vector2 = newBody.Shape.VertexData.Positions[nextIndex]
				D            float64 = MathCrossVector2(p1, p2)
				triangleArea float64 = D / 2
			)
			area += triangleArea
			center.X += float64(float64(triangleArea) * 1.0 / 3.0 * float64(p1.X+p2.X))
			center.Y += float64(float64(triangleArea) * 1.0 / 3.0 * float64(p1.Y+p2.Y))
			var intx2 float64 = p1.X*p1.X + p2.X*p1.X + p2.X*p2.X
			var inty2 float64 = p1.Y*p1.Y + p2.Y*p1.Y + p2.Y*p2.Y
			inertia += float64((float64(D) * (0.25 * 1.0 / 3.0)) * float64(intx2+inty2))
		}
		center.X *= float64(1.0 / float64(area))
		center.Y *= float64(1.0 / float64(area))
		for i := int(0); i < int(newBody.Shape.VertexData.VertexCount); i++ {
			newBody.Shape.VertexData.Positions[i].X -= center.X
			newBody.Shape.VertexData.Positions[i].Y -= center.Y
		}
		newBody.Mass = density * area
		if float64(newBody.Mass) != 0.0 {
			newBody.InverseMass = float64(1.0 / float64(newBody.Mass))
		} else {
			newBody.InverseMass = 0.0
		}
		newBody.Inertia = density * inertia
		if float64(newBody.Inertia) != 0.0 {
			newBody.InverseInertia = float64(1.0 / float64(newBody.Inertia))
		} else {
			newBody.InverseInertia = 0.0
		}
		newBody.StaticFriction = 0.4
		newBody.DynamicFriction = 0.2
		newBody.Restitution = 0.0
		newBody.UseGravity = bool(true)
		newBody.IsGrounded = bool(false)
		newBody.FreezeOrient = bool(false)
		bodies[physicsBodiesCount] = newBody
		physicsBodiesCount++
	}
	return newBody
}
func CreatePhysicsBodyPolygon(pos Vector2, radius float64, sides int, density float64) PhysicsBody {
	var newBody PhysicsBody = new(PhysicsBodyData)
	var newId int = FindAvailableBodyIndex()
	if newId != -1 {
		newBody.Id = uint(newId)
		newBody.Enabled = bool(true)
		newBody.Position = pos
		newBody.Velocity = Vector2{X: 0.0, Y: 0.0}
		newBody.Force = Vector2{X: 0.0, Y: 0.0}
		newBody.AngularVelocity = 0.0
		newBody.Torque = 0.0
		newBody.Orient = 0.0
		newBody.Shape.Type = PhysicsShapeType(PHYSICS_POLYGON)
		newBody.Shape.Body = newBody
		newBody.Shape.Transform = Mat2Radians(0.0)
		newBody.Shape.VertexData = CreateRandomPolygon(radius, sides)
		var center Vector2 = Vector2{X: 0.0, Y: 0.0}
		var area float64 = 0.0
		var inertia float64 = 0.0
		for i := int(0); i < int(newBody.Shape.VertexData.VertexCount); i++ {
			var (
				position1 Vector2 = newBody.Shape.VertexData.Positions[i]
				nextIndex int     = (func() int {
					if (i + 1) < int(newBody.Shape.VertexData.VertexCount) {
						return i + 1
					}
					return 0
				}())
				position2    Vector2 = newBody.Shape.VertexData.Positions[nextIndex]
				cross        float64 = MathCrossVector2(position1, position2)
				triangleArea float64 = cross / 2
			)
			area += triangleArea
			center.X += float64(float64(triangleArea) * 1.0 / 3.0 * float64(position1.X+position2.X))
			center.Y += float64(float64(triangleArea) * 1.0 / 3.0 * float64(position1.Y+position2.Y))
			var intx2 float64 = position1.X*position1.X + position2.X*position1.X + position2.X*position2.X
			var inty2 float64 = position1.Y*position1.Y + position2.Y*position1.Y + position2.Y*position2.Y
			inertia += float64((float64(cross) * (0.25 * 1.0 / 3.0)) * float64(intx2+inty2))
		}
		center.X *= float64(1.0 / float64(area))
		center.Y *= float64(1.0 / float64(area))
		for i := int(0); i < int(newBody.Shape.VertexData.VertexCount); i++ {
			newBody.Shape.VertexData.Positions[i].X -= center.X
			newBody.Shape.VertexData.Positions[i].Y -= center.Y
		}
		newBody.Mass = density * area
		if float64(newBody.Mass) != 0.0 {
			newBody.InverseMass = float64(1.0 / float64(newBody.Mass))
		} else {
			newBody.InverseMass = 0.0
		}
		newBody.Inertia = density * inertia
		if float64(newBody.Inertia) != 0.0 {
			newBody.InverseInertia = float64(1.0 / float64(newBody.Inertia))
		} else {
			newBody.InverseInertia = 0.0
		}
		newBody.StaticFriction = 0.4
		newBody.DynamicFriction = 0.2
		newBody.Restitution = 0.0
		newBody.UseGravity = bool(true)
		newBody.IsGrounded = bool(false)
		newBody.FreezeOrient = bool(false)
		bodies[physicsBodiesCount] = newBody
		physicsBodiesCount++
	}
	return newBody
}
func PhysicsAddForce(body PhysicsBody, force Vector2) {
	if body != nil {
		body.Force = Vector2Add(body.Force, force)
	}
}
func PhysicsAddTorque(body PhysicsBody, amount float64) {
	if body != nil {
		body.Torque += amount
	}
}
func PhysicsShatter(body PhysicsBody, position Vector2, force float64) {
	if body != nil {
		if body.Shape.Type == PhysicsShapeType(PHYSICS_POLYGON) {
			var (
				vertexData PolygonData = body.Shape.VertexData
				collision  bool        = bool(false)
			)
			for i := int(0); i < int(vertexData.VertexCount); i++ {
				var (
					positionA Vector2 = body.Position
					positionB Vector2 = Mat2MultiplyVector2(body.Shape.Transform, Vector2Add(body.Position, vertexData.Positions[i]))
					nextIndex int     = (func() int {
						if (i + 1) < int(vertexData.VertexCount) {
							return i + 1
						}
						return 0
					}())
					positionC Vector2 = Mat2MultiplyVector2(body.Shape.Transform, Vector2Add(body.Position, vertexData.Positions[nextIndex]))
					alpha     float64 = ((positionB.Y-positionC.Y)*(position.X-positionC.X) + (positionC.X-positionB.X)*(position.Y-positionC.Y)) / ((positionB.Y-positionC.Y)*(positionA.X-positionC.X) + (positionC.X-positionB.X)*(positionA.Y-positionC.Y))
					beta      float64 = ((positionC.Y-positionA.Y)*(position.X-positionC.X) + (positionA.X-positionC.X)*(position.Y-positionC.Y)) / ((positionB.Y-positionC.Y)*(positionA.X-positionC.X) + (positionC.X-positionB.X)*(positionA.Y-positionC.Y))
					gamma     float64 = float64(1.0 - float64(alpha) - float64(beta))
				)
				if float64(alpha) > 0.0 && (float64(beta) > 0.0) && (float64(gamma) > 0.0) {
					collision = bool(true)
					break
				}
			}
			if collision {
				var (
					count    int       = int(vertexData.VertexCount)
					bodyPos  Vector2   = body.Position
					vertices []Vector2 = make([]Vector2, count)
					trans    Mat2      = body.Shape.Transform
				)
				for i := int(0); i < count; i++ {
					vertices[i] = vertexData.Positions[i]
				}
				DestroyPhysicsBody(body)
				for i := int(0); i < count; i++ {
					var (
						nextIndex int = (func() int {
							if (i + 1) < count {
								return i + 1
							}
							return 0
						}())
						center Vector2 = TriangleBarycenter(vertices[i], vertices[nextIndex], Vector2{X: 0.0, Y: 0.0})
					)
					center = Vector2Add(bodyPos, center)
					var offset Vector2 = Vector2Subtract(center, bodyPos)
					var newBody PhysicsBody = CreatePhysicsBodyPolygon(center, 10, 3, 10)
					var newData PolygonData = PolygonData{}
					newData.VertexCount = 3
					newData.Positions[0] = Vector2Subtract(vertices[i], offset)
					newData.Positions[1] = Vector2Subtract(vertices[nextIndex], offset)
					newData.Positions[2] = Vector2Subtract(position, center)
					newData.Positions[0].X *= 0.95
					newData.Positions[0].Y *= 0.95
					newData.Positions[1].X *= 0.95
					newData.Positions[1].Y *= 0.95
					newData.Positions[2].X *= 0.95
					newData.Positions[2].Y *= 0.95
					for j := int(0); j < int(newData.VertexCount); j++ {
						var (
							nextVertex int = (func() int {
								if (j + 1) < int(newData.VertexCount) {
									return j + 1
								}
								return 0
							}())
							face Vector2 = Vector2Subtract(newData.Positions[nextVertex], newData.Positions[j])
						)
						newData.Normals[j] = Vector2{X: face.Y, Y: -face.X}
						MathNormalize(&newData.Normals[j])
					}
					newBody.Shape.VertexData = newData
					newBody.Shape.Transform = trans
					center = Vector2{X: 0.0, Y: 0.0}
					var area float64 = 0.0
					var inertia float64 = 0.0
					for j := int(0); j < int(newBody.Shape.VertexData.VertexCount); j++ {
						var (
							p1         Vector2 = newBody.Shape.VertexData.Positions[j]
							nextVertex int     = (func() int {
								if (j + 1) < int(newBody.Shape.VertexData.VertexCount) {
									return j + 1
								}
								return 0
							}())
							p2           Vector2 = newBody.Shape.VertexData.Positions[nextVertex]
							D            float64 = MathCrossVector2(p1, p2)
							triangleArea float64 = D / 2
						)
						area += triangleArea
						center.X += float64(float64(triangleArea) * 1.0 / 3.0 * float64(p1.X+p2.X))
						center.Y += float64(float64(triangleArea) * 1.0 / 3.0 * float64(p1.Y+p2.Y))
						var intx2 float64 = p1.X*p1.X + p2.X*p1.X + p2.X*p2.X
						var inty2 float64 = p1.Y*p1.Y + p2.Y*p1.Y + p2.Y*p2.Y
						inertia += float64((float64(D) * (0.25 * 1.0 / 3.0)) * float64(intx2+inty2))
					}
					center.X *= float64(1.0 / float64(area))
					center.Y *= float64(1.0 / float64(area))
					newBody.Mass = area
					if float64(newBody.Mass) != 0.0 {
						newBody.InverseMass = float64(1.0 / float64(newBody.Mass))
					} else {
						newBody.InverseMass = 0.0
					}
					newBody.Inertia = inertia
					if float64(newBody.Inertia) != 0.0 {
						newBody.InverseInertia = float64(1.0 / float64(newBody.Inertia))
					} else {
						newBody.InverseInertia = 0.0
					}
					var pointA Vector2 = newBody.Position
					var pointB Vector2 = Vector2Subtract(newData.Positions[1], newData.Positions[0])
					pointB.X /= 2.0
					pointB.Y /= 2.0
					var forceDirection Vector2 = Vector2Subtract(Vector2Add(pointA, Vector2Add(newData.Positions[0], pointB)), newBody.Position)
					MathNormalize(&forceDirection)
					forceDirection.X *= force
					forceDirection.Y *= force
					PhysicsAddForce(newBody, forceDirection)
				}
			}
		}
	}
}
func GetPhysicsBodiesCount() int {
	return int(physicsBodiesCount)
}
func GetPhysicsBody(index int) PhysicsBody {
	// if index < int(physicsBodiesCount) {
	// 	if bodies[index] == nil {
	// 	}
	// }
	return bodies[index]
}
func GetPhysicsShapeType(index int) int {
	var result int = -1
	if index < int(physicsBodiesCount) {
		if bodies[index] != nil {
			result = int(bodies[index].Shape.Type)
		}
	}
	return result
}
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
func GetPhysicsShapeVertex(body PhysicsBody, vertex int) Vector2 {
	var position Vector2 = Vector2{X: 0.0, Y: 0.0}
	if body != nil {
		switch body.Shape.Type {
		case PHYSICS_CIRCLE:
			position.X = body.Position.X + math.Cos(float64(360.0/PHYSAC_CIRCLE_VERTICES*float64(vertex)*(PHYSAC_PI/180.0)))*body.Shape.Radius
			position.Y = body.Position.Y + math.Sin(float64(360.0/PHYSAC_CIRCLE_VERTICES*float64(vertex)*(PHYSAC_PI/180.0)))*body.Shape.Radius
		case PHYSICS_POLYGON:
			var vertexData PolygonData = body.Shape.VertexData
			position = Vector2Add(body.Position, Mat2MultiplyVector2(body.Shape.Transform, vertexData.Positions[vertex]))
		default:
		}
	}
	return position
}
func SetPhysicsBodyRotation(body PhysicsBody, radians float64) {
	if body != nil {
		body.Orient = radians
		if body.Shape.Type == PhysicsShapeType(PHYSICS_POLYGON) {
			body.Shape.Transform = Mat2Radians(radians)
		}
	}
}
func DestroyPhysicsBody(body PhysicsBody) {
	if body != nil {
		var (
			id    int = int(body.Id)
			index int = -1
		)
		for i := int(0); i < int(physicsBodiesCount); i++ {
			if bodies[i].Id == uint(id) {
				index = i
				break
			}
		}
		if index == -1 {
			return
		}
		bodies[index] = nil
		for i := int(index); i < int(physicsBodiesCount); i++ {
			if (i + 1) < int(physicsBodiesCount) {
				bodies[i] = bodies[i+1]
			}
		}
		physicsBodiesCount--
	}
}
func ClosePhysics() {
	physicsThreadEnabled = bool(false)
	for i := int(physicsManifoldsCount - 1); i >= 0; i-- {
		DestroyPhysicsManifold(contacts[i])
	}
	for i := int(physicsBodiesCount - 1); i >= 0; i-- {
		DestroyPhysicsBody(bodies[i])
	}
}
func FindAvailableBodyIndex() int {
	var index int = -1
	for i := int(0); i < PHYSAC_MAX_BODIES; i++ {
		var currentId int = i
		for k := int(0); k < int(physicsBodiesCount); k++ {
			if bodies[k].Id == uint(currentId) {
				currentId++
				break
			}
		}
		if currentId == i {
			index = i
			break
		}
	}
	return index
}
func CreateRandomPolygon(radius float64, sides int) PolygonData {
	var data PolygonData = PolygonData{}
	data.VertexCount = uint(sides)
	for i := int(0); i < int(data.VertexCount); i++ {
		data.Positions[i].X = math.Cos(float64(360.0/float64(sides)*float64(i)*(PHYSAC_PI/180.0))) * radius
		data.Positions[i].Y = math.Sin(float64(360.0/float64(sides)*float64(i)*(PHYSAC_PI/180.0))) * radius
	}
	for i := int(0); i < int(data.VertexCount); i++ {
		var (
			nextIndex int = (func() int {
				if (i + 1) < sides {
					return i + 1
				}
				return 0
			}())
			face Vector2 = Vector2Subtract(data.Positions[nextIndex], data.Positions[i])
		)
		data.Normals[i] = Vector2{X: face.Y, Y: -face.X}
		MathNormalize(&data.Normals[i])
	}
	return data
}
func CreateRectanglePolygon(pos Vector2, size Vector2) PolygonData {
	var data PolygonData = PolygonData{}
	data.VertexCount = 4
	data.Positions[0] = Vector2{X: pos.X + size.X/2, Y: pos.Y - size.Y/2}
	data.Positions[1] = Vector2{X: pos.X + size.X/2, Y: pos.Y + size.Y/2}
	data.Positions[2] = Vector2{X: pos.X - size.X/2, Y: pos.Y + size.Y/2}
	data.Positions[3] = Vector2{X: pos.X - size.X/2, Y: pos.Y - size.Y/2}
	for i := int(0); i < int(data.VertexCount); i++ {
		var (
			nextIndex int = (func() int {
				if (i + 1) < int(data.VertexCount) {
					return i + 1
				}
				return 0
			}())
			face Vector2 = Vector2Subtract(data.Positions[nextIndex], data.Positions[i])
		)
		data.Normals[i] = Vector2{X: face.Y, Y: -face.X}
		MathNormalize(&data.Normals[i])
	}
	return data
}
func PhysicsStep() {
	stepsCount++
	for i := int(physicsManifoldsCount - 1); i >= 0; i-- {
		var manifold PhysicsManifold = contacts[i]
		if manifold != nil {
			DestroyPhysicsManifold(manifold)
		}
	}
	for i := int(0); i < int(physicsBodiesCount); i++ {
		var body PhysicsBody = bodies[i]
		body.IsGrounded = bool(false)
	}
	for i := int(0); i < int(physicsBodiesCount); i++ {
		var bodyA PhysicsBody = bodies[i]
		if bodyA != nil {
			for j := int(i + 1); j < int(physicsBodiesCount); j++ {
				var bodyB PhysicsBody = bodies[j]
				if bodyB != nil {
					if bodyA.InverseMass == 0 && bodyB.InverseMass == 0 {
						continue
					}
					var manifold PhysicsManifold = CreatePhysicsManifold(bodyA, bodyB)
					SolvePhysicsManifold(manifold)
					if manifold.ContactsCount > 0 {
						var newManifold PhysicsManifold = CreatePhysicsManifold(bodyA, bodyB)
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
		}
	}
	for i := int(0); i < int(physicsBodiesCount); i++ {
		var body PhysicsBody = bodies[i]
		if body != nil {
			IntegratePhysicsForces(body)
		}
	}
	for i := int(0); i < int(physicsManifoldsCount); i++ {
		var manifold PhysicsManifold = contacts[i]
		if manifold != nil {
			InitializePhysicsManifolds(manifold)
		}
	}
	for i := int(0); i < PHYSAC_COLLISION_ITERATIONS; i++ {
		for j := int(0); j < int(physicsManifoldsCount); j++ {
			var manifold PhysicsManifold = contacts[i]
			if manifold != nil {
				IntegratePhysicsImpulses(manifold)
			}
		}
	}
	for i := int(0); i < int(physicsBodiesCount); i++ {
		var body PhysicsBody = bodies[i]
		if body != nil {
			IntegratePhysicsVelocity(body)
		}
	}
	for i := int(0); i < int(physicsManifoldsCount); i++ {
		var manifold PhysicsManifold = contacts[i]
		if manifold != nil {
			CorrectPhysicsPositions(manifold)
		}
	}
	for i := int(0); i < int(physicsBodiesCount); i++ {
		var body PhysicsBody = bodies[i]
		if body != nil {
			body.Force = Vector2{X: 0.0, Y: 0.0}
			body.Torque = 0.0
		}
	}
}
func RunPhysicsStep() {
	currentTime = GetCurrentTime()
	var delta float64 = currentTime - startTime
	accumulator += delta
	for accumulator >= deltaTime {
		PhysicsStep()
		accumulator -= deltaTime
	}
	startTime = currentTime
}
func SetPhysicsTimeStep(delta float64) {
	deltaTime = delta
}
func FindAvailableManifoldIndex() int {
	var index int = -1
	for i := int(0); i < PHYSAC_MAX_MANIFOLDS; i++ {
		var currentId int = i
		for k := int(0); k < int(physicsManifoldsCount); k++ {
			if contacts[k].Id == uint(currentId) {
				currentId++
				break
			}
		}
		if currentId == i {
			index = i
			break
		}
	}
	return index
}
func CreatePhysicsManifold(a PhysicsBody, b PhysicsBody) PhysicsManifold {
	var newManifold PhysicsManifold = new(PhysicsManifoldData)
	var newId int = FindAvailableManifoldIndex()
	if newId != -1 {
		newManifold.Id = uint(newId)
		newManifold.BodyA = a
		newManifold.BodyB = b
		newManifold.Penetration = 0
		newManifold.Normal = Vector2{X: 0.0, Y: 0.0}
		newManifold.Contacts[0] = Vector2{X: 0.0, Y: 0.0}
		newManifold.Contacts[1] = Vector2{X: 0.0, Y: 0.0}
		newManifold.ContactsCount = 0
		newManifold.Restitution = 0.0
		newManifold.DynamicFriction = 0.0
		newManifold.StaticFriction = 0.0
		contacts[physicsManifoldsCount] = newManifold
		physicsManifoldsCount++
	}
	return newManifold
}
func DestroyPhysicsManifold(manifold PhysicsManifold) {
	if manifold != nil {
		var (
			id    int = int(manifold.Id)
			index int = -1
		)
		for i := int(0); i < int(physicsManifoldsCount); i++ {
			if contacts[i].Id == uint(id) {
				index = i
				break
			}
		}
		if index == -1 {
			return
		}
		contacts[index] = nil
		for i := int(index); i < int(physicsManifoldsCount); i++ {
			if (i + 1) < int(physicsManifoldsCount) {
				contacts[i] = contacts[i+1]
			}
		}
		physicsManifoldsCount--
	}
}
func SolvePhysicsManifold(manifold PhysicsManifold) {
	switch manifold.BodyA.Shape.Type {
	case PHYSICS_CIRCLE:
		switch manifold.BodyB.Shape.Type {
		case PHYSICS_CIRCLE:
			SolveCircleToCircle(manifold)
		case PHYSICS_POLYGON:
			SolveCircleToPolygon(manifold)
		default:
		}
	case PHYSICS_POLYGON:
		switch manifold.BodyB.Shape.Type {
		case PHYSICS_CIRCLE:
			SolvePolygonToCircle(manifold)
		case PHYSICS_POLYGON:
			SolvePolygonToPolygon(manifold)
		default:
		}
	default:
	}
	if !manifold.BodyB.IsGrounded {
		manifold.BodyB.IsGrounded = manifold.Normal.Y < 0
	}
}
func SolveCircleToCircle(manifold PhysicsManifold) {
	var (
		bodyA PhysicsBody = manifold.BodyA
		bodyB PhysicsBody = manifold.BodyB
	)
	if bodyA == nil || bodyB == nil {
		return
	}
	var normal Vector2 = Vector2Subtract(bodyB.Position, bodyA.Position)
	var distSqr float64 = MathLenSqr(normal)
	var radius float64 = bodyA.Shape.Radius + bodyB.Shape.Radius
	if distSqr >= radius*radius {
		manifold.ContactsCount = 0
		return
	}
	var distance float64 = math.Sqrt(distSqr)
	manifold.ContactsCount = 1
	if float64(distance) == 0.0 {
		manifold.Penetration = bodyA.Shape.Radius
		manifold.Normal = Vector2{X: 1.0, Y: 0.0}
		manifold.Contacts[0] = bodyA.Position
	} else {
		manifold.Penetration = radius - distance
		manifold.Normal = Vector2{X: normal.X / distance, Y: normal.Y / distance}
		manifold.Contacts[0] = Vector2{X: manifold.Normal.X*bodyA.Shape.Radius + bodyA.Position.X, Y: manifold.Normal.Y*bodyA.Shape.Radius + bodyA.Position.Y}
	}
	if !bodyA.IsGrounded {
		bodyA.IsGrounded = manifold.Normal.Y < 0
	}
}
func SolveCircleToPolygon(manifold PhysicsManifold) {
	var (
		bodyA PhysicsBody = manifold.BodyA
		bodyB PhysicsBody = manifold.BodyB
	)
	if bodyA == nil || bodyB == nil {
		return
	}
	SolveDifferentShapes(manifold, bodyA, bodyB)
}
func SolvePolygonToCircle(manifold PhysicsManifold) {
	var (
		bodyA PhysicsBody = manifold.BodyA
		bodyB PhysicsBody = manifold.BodyB
	)
	if bodyA == nil || bodyB == nil {
		return
	}
	SolveDifferentShapes(manifold, bodyB, bodyA)
	manifold.Normal.X *= -1.0
	manifold.Normal.Y *= -1.0
}
func SolveDifferentShapes(manifold PhysicsManifold, bodyA PhysicsBody, bodyB PhysicsBody) {
	manifold.ContactsCount = 0
	var center Vector2 = bodyA.Position
	center = Mat2MultiplyVector2(Mat2Transpose(bodyB.Shape.Transform), Vector2Subtract(center, bodyB.Position))
	var separation float64 = float64(-PHYSAC_FLT_MAX)
	var faceNormal int = 0
	var vertexData PolygonData = bodyB.Shape.VertexData
	for i := int(0); i < int(vertexData.VertexCount); i++ {
		var currentSeparation float64 = MathDot(vertexData.Normals[i], Vector2Subtract(center, vertexData.Positions[i]))
		if currentSeparation > bodyA.Shape.Radius {
			return
		}
		if currentSeparation > separation {
			separation = currentSeparation
			faceNormal = i
		}
	}
	var v1 Vector2 = vertexData.Positions[faceNormal]
	var nextIndex int = (func() int {
		if (faceNormal + 1) < int(vertexData.VertexCount) {
			return faceNormal + 1
		}
		return 0
	}())
	var v2 Vector2 = vertexData.Positions[nextIndex]
	if float64(separation) < PHYSAC_EPSILON {
		manifold.ContactsCount = 1
		var normal Vector2 = Mat2MultiplyVector2(bodyB.Shape.Transform, vertexData.Normals[faceNormal])
		manifold.Normal = Vector2{X: -normal.X, Y: -normal.Y}
		manifold.Contacts[0] = Vector2{X: manifold.Normal.X*bodyA.Shape.Radius + bodyA.Position.X, Y: manifold.Normal.Y*bodyA.Shape.Radius + bodyA.Position.Y}
		manifold.Penetration = bodyA.Shape.Radius
		return
	}
	var dot1 float64 = MathDot(Vector2Subtract(center, v1), Vector2Subtract(v2, v1))
	var dot2 float64 = MathDot(Vector2Subtract(center, v2), Vector2Subtract(v1, v2))
	manifold.Penetration = bodyA.Shape.Radius - separation
	if float64(dot1) <= 0.0 {
		if DistSqr(center, v1) > bodyA.Shape.Radius*bodyA.Shape.Radius {
			return
		}
		manifold.ContactsCount = 1
		var normal Vector2 = Vector2Subtract(v1, center)
		normal = Mat2MultiplyVector2(bodyB.Shape.Transform, normal)
		MathNormalize(&normal)
		manifold.Normal = normal
		v1 = Mat2MultiplyVector2(bodyB.Shape.Transform, v1)
		v1 = Vector2Add(v1, bodyB.Position)
		manifold.Contacts[0] = v1
	} else if float64(dot2) <= 0.0 {
		if DistSqr(center, v2) > bodyA.Shape.Radius*bodyA.Shape.Radius {
			return
		}
		manifold.ContactsCount = 1
		var normal Vector2 = Vector2Subtract(v2, center)
		v2 = Mat2MultiplyVector2(bodyB.Shape.Transform, v2)
		v2 = Vector2Add(v2, bodyB.Position)
		manifold.Contacts[0] = v2
		normal = Mat2MultiplyVector2(bodyB.Shape.Transform, normal)
		MathNormalize(&normal)
		manifold.Normal = normal
	} else {
		var normal Vector2 = vertexData.Normals[faceNormal]
		if MathDot(Vector2Subtract(center, v1), normal) > bodyA.Shape.Radius {
			return
		}
		normal = Mat2MultiplyVector2(bodyB.Shape.Transform, normal)
		manifold.Normal = Vector2{X: -normal.X, Y: -normal.Y}
		manifold.Contacts[0] = Vector2{X: manifold.Normal.X*bodyA.Shape.Radius + bodyA.Position.X, Y: manifold.Normal.Y*bodyA.Shape.Radius + bodyA.Position.Y}
		manifold.ContactsCount = 1
	}
}
func SolvePolygonToPolygon(manifold PhysicsManifold) {
	if manifold.BodyA == nil || manifold.BodyB == nil {
		return
	}
	var bodyA PhysicsShape = manifold.BodyA.Shape
	var bodyB PhysicsShape = manifold.BodyB.Shape
	manifold.ContactsCount = 0
	var faceA int = 0
	var penetrationA float64 = FindAxisLeastPenetration(&faceA, bodyA, bodyB)
	if float64(penetrationA) >= 0.0 {
		return
	}
	var faceB int = 0
	var penetrationB float64 = FindAxisLeastPenetration(&faceB, bodyB, bodyA)
	if float64(penetrationB) >= 0.0 {
		return
	}
	var referenceIndex int = 0
	var flip bool = bool(false)
	var refPoly PhysicsShape
	var incPoly PhysicsShape
	if BiasGreaterThan(penetrationA, penetrationB) {
		refPoly = bodyA
		incPoly = bodyB
		referenceIndex = faceA
	} else {
		refPoly = bodyB
		incPoly = bodyA
		referenceIndex = faceB
		flip = bool(true)
	}
	var incidentFace [2]Vector2
	FindIncidentFace(&incidentFace[0], &incidentFace[1], refPoly, incPoly, referenceIndex)
	var refData PolygonData = refPoly.VertexData
	var v1 Vector2 = refData.Positions[referenceIndex]
	if (referenceIndex + 1) < int(refData.VertexCount) {
		referenceIndex = referenceIndex + 1
	} else {
		referenceIndex = 0
	}
	var v2 Vector2 = refData.Positions[referenceIndex]
	v1 = Mat2MultiplyVector2(refPoly.Transform, v1)
	v1 = Vector2Add(v1, refPoly.Body.Position)
	v2 = Mat2MultiplyVector2(refPoly.Transform, v2)
	v2 = Vector2Add(v2, refPoly.Body.Position)
	var sidePlaneNormal Vector2 = Vector2Subtract(v2, v1)
	MathNormalize(&sidePlaneNormal)
	var refFaceNormal Vector2 = Vector2{X: sidePlaneNormal.Y, Y: -sidePlaneNormal.X}
	var refC float64 = MathDot(refFaceNormal, v1)
	var negSide float64 = MathDot(sidePlaneNormal, v1) * float64(-1)
	var posSide float64 = MathDot(sidePlaneNormal, v2)
	if Clip(Vector2{X: -sidePlaneNormal.X, Y: -sidePlaneNormal.Y}, negSide, &incidentFace[0], &incidentFace[1]) < 2 {
		return
	}
	if Clip(sidePlaneNormal, posSide, &incidentFace[0], &incidentFace[1]) < 2 {
		return
	}
	if flip {
		manifold.Normal = Vector2{X: -refFaceNormal.X, Y: -refFaceNormal.Y}
	} else {
		manifold.Normal = refFaceNormal
	}
	var currentPoint int = 0
	var separation float64 = MathDot(refFaceNormal, incidentFace[0]) - refC
	if float64(separation) <= 0.0 {
		manifold.Contacts[currentPoint] = incidentFace[0]
		manifold.Penetration = -separation
		currentPoint++
	} else {
		manifold.Penetration = 0.0
	}
	separation = MathDot(refFaceNormal, incidentFace[1]) - refC
	if float64(separation) <= 0.0 {
		manifold.Contacts[currentPoint] = incidentFace[1]
		manifold.Penetration += -separation
		currentPoint++
		manifold.Penetration /= float64(currentPoint)
	}
	manifold.ContactsCount = uint(currentPoint)
}
func IntegratePhysicsForces(body PhysicsBody) {
	if body == nil || float64(body.InverseMass) == 0.0 || !body.Enabled {
		return
	}
	body.Velocity.X += float64(float64(body.Force.X*body.InverseMass) * (deltaTime / 2.0))
	body.Velocity.Y += float64(float64(body.Force.Y*body.InverseMass) * (deltaTime / 2.0))
	if body.UseGravity {
		body.Velocity.X += float64(float64(gravityForce.X) * (deltaTime / 1000 / 2.0))
		body.Velocity.Y += float64(float64(gravityForce.Y) * (deltaTime / 1000 / 2.0))
	}
	if body.FreezeOrient {
		body.AngularVelocity += float64(float64(body.Torque*body.InverseInertia) * (deltaTime / 2.0))
	}
}
func InitializePhysicsManifolds(manifold PhysicsManifold) {
	var (
		bodyA PhysicsBody = manifold.BodyA
		bodyB PhysicsBody = manifold.BodyB
	)
	if bodyA == nil || bodyB == nil {
		return
	}
	manifold.Restitution = math.Sqrt(bodyA.Restitution * bodyB.Restitution)
	manifold.StaticFriction = math.Sqrt(bodyA.StaticFriction * bodyB.StaticFriction)
	manifold.DynamicFriction = math.Sqrt(bodyA.DynamicFriction * bodyB.DynamicFriction)
	for i := int(0); i < int(manifold.ContactsCount); i++ {
		var (
			radiusA Vector2 = Vector2Subtract(manifold.Contacts[i], bodyA.Position)
			radiusB Vector2 = Vector2Subtract(manifold.Contacts[i], bodyB.Position)
			crossA  Vector2 = MathCross(bodyA.AngularVelocity, radiusA)
			crossB  Vector2 = MathCross(bodyB.AngularVelocity, radiusB)
			radiusV Vector2 = Vector2{X: 0.0, Y: 0.0}
		)
		radiusV.X = bodyB.Velocity.X + crossB.X - bodyA.Velocity.X - crossA.X
		radiusV.Y = bodyB.Velocity.Y + crossB.Y - bodyA.Velocity.Y - crossA.Y
		if float64(MathLenSqr(radiusV)) < (float64(MathLenSqr(Vector2{X: float64(float64(gravityForce.X) * deltaTime / 1000), Y: float64(float64(gravityForce.Y) * deltaTime / 1000)})) + PHYSAC_EPSILON) {
			manifold.Restitution = 0
		}
	}
}
func IntegratePhysicsImpulses(manifold PhysicsManifold) {
	var (
		bodyA PhysicsBody = manifold.BodyA
		bodyB PhysicsBody = manifold.BodyB
	)
	if bodyA == nil || bodyB == nil {
		return
	}
	if math.Abs(float64(bodyA.InverseMass+bodyB.InverseMass)) <= PHYSAC_EPSILON {
		bodyA.Velocity = Vector2{X: 0.0, Y: 0.0}
		bodyB.Velocity = Vector2{X: 0.0, Y: 0.0}
		return
	}
	for i := int(0); i < int(manifold.ContactsCount); i++ {
		var (
			radiusA Vector2 = Vector2Subtract(manifold.Contacts[i], bodyA.Position)
			radiusB Vector2 = Vector2Subtract(manifold.Contacts[i], bodyB.Position)
			radiusV Vector2 = Vector2{X: 0.0, Y: 0.0}
		)
		radiusV.X = bodyB.Velocity.X + MathCross(bodyB.AngularVelocity, radiusB).X - bodyA.Velocity.X - MathCross(bodyA.AngularVelocity, radiusA).X
		radiusV.Y = bodyB.Velocity.Y + MathCross(bodyB.AngularVelocity, radiusB).Y - bodyA.Velocity.Y - MathCross(bodyA.AngularVelocity, radiusA).Y
		var contactVelocity float64 = MathDot(radiusV, manifold.Normal)
		if float64(contactVelocity) > 0.0 {
			return
		}
		var raCrossN float64 = MathCrossVector2(radiusA, manifold.Normal)
		var rbCrossN float64 = MathCrossVector2(radiusB, manifold.Normal)
		var inverseMassSum float64 = bodyA.InverseMass + bodyB.InverseMass + (raCrossN*raCrossN)*bodyA.InverseInertia + (rbCrossN*rbCrossN)*bodyB.InverseInertia
		var impulse float64 = float64(-(float64(manifold.Restitution) + 1.0)) * contactVelocity
		impulse /= inverseMassSum
		impulse /= float64(manifold.ContactsCount)
		var impulseV Vector2 = Vector2{X: manifold.Normal.X * impulse, Y: manifold.Normal.Y * impulse}
		if bodyA.Enabled {
			bodyA.Velocity.X += bodyA.InverseMass * (-impulseV.X)
			bodyA.Velocity.Y += bodyA.InverseMass * (-impulseV.Y)
			if !bodyA.FreezeOrient {
				bodyA.AngularVelocity += bodyA.InverseInertia * MathCrossVector2(radiusA, Vector2{X: -impulseV.X, Y: -impulseV.Y})
			}
		}
		if bodyB.Enabled {
			bodyB.Velocity.X += bodyB.InverseMass * impulseV.X
			bodyB.Velocity.Y += bodyB.InverseMass * impulseV.Y
			if !bodyB.FreezeOrient {
				bodyB.AngularVelocity += bodyB.InverseInertia * MathCrossVector2(radiusB, impulseV)
			}
		}
		radiusV.X = bodyB.Velocity.X + MathCross(bodyB.AngularVelocity, radiusB).X - bodyA.Velocity.X - MathCross(bodyA.AngularVelocity, radiusA).X
		radiusV.Y = bodyB.Velocity.Y + MathCross(bodyB.AngularVelocity, radiusB).Y - bodyA.Velocity.Y - MathCross(bodyA.AngularVelocity, radiusA).Y
		var tangent Vector2 = Vector2{X: radiusV.X - manifold.Normal.X*MathDot(radiusV, manifold.Normal), Y: radiusV.Y - manifold.Normal.Y*MathDot(radiusV, manifold.Normal)}
		MathNormalize(&tangent)
		var impulseTangent float64 = -MathDot(radiusV, tangent)
		impulseTangent /= inverseMassSum
		impulseTangent /= float64(manifold.ContactsCount)
		var absImpulseTangent float64 = float64(math.Abs(float64(impulseTangent)))
		if float64(absImpulseTangent) <= PHYSAC_EPSILON {
			return
		}
		var tangentImpulse Vector2 = Vector2{X: 0.0, Y: 0.0}
		if absImpulseTangent < impulse*manifold.StaticFriction {
			tangentImpulse = Vector2{X: tangent.X * impulseTangent, Y: tangent.Y * impulseTangent}
		} else {
			tangentImpulse = Vector2{X: tangent.X * (-impulse) * manifold.DynamicFriction, Y: tangent.Y * (-impulse) * manifold.DynamicFriction}
		}
		if bodyA.Enabled {
			bodyA.Velocity.X += bodyA.InverseMass * (-tangentImpulse.X)
			bodyA.Velocity.Y += bodyA.InverseMass * (-tangentImpulse.Y)
			if !bodyA.FreezeOrient {
				bodyA.AngularVelocity += bodyA.InverseInertia * MathCrossVector2(radiusA, Vector2{X: -tangentImpulse.X, Y: -tangentImpulse.Y})
			}
		}
		if bodyB.Enabled {
			bodyB.Velocity.X += bodyB.InverseMass * tangentImpulse.X
			bodyB.Velocity.Y += bodyB.InverseMass * tangentImpulse.Y
			if !bodyB.FreezeOrient {
				bodyB.AngularVelocity += bodyB.InverseInertia * MathCrossVector2(radiusB, tangentImpulse)
			}
		}
	}
}
func IntegratePhysicsVelocity(body PhysicsBody) {
	if body == nil || !body.Enabled {
		return
	}
	body.Position.X += float64(float64(body.Velocity.X) * deltaTime)
	body.Position.Y += float64(float64(body.Velocity.Y) * deltaTime)
	if !body.FreezeOrient {
		body.Orient += float64(float64(body.AngularVelocity) * deltaTime)
	}
	Mat2Set(&body.Shape.Transform, body.Orient)
	IntegratePhysicsForces(body)
}
func CorrectPhysicsPositions(manifold PhysicsManifold) {
	var (
		bodyA PhysicsBody = manifold.BodyA
		bodyB PhysicsBody = manifold.BodyB
	)
	if bodyA == nil || bodyB == nil {
		return
	}
	var correction Vector2 = Vector2{X: 0.0, Y: 0.0}
	correction.X = float64(((func() float64 {
		if (float64(manifold.Penetration) - PHYSAC_PENETRATION_ALLOWANCE) > 0.0 {
			return float64(manifold.Penetration) - PHYSAC_PENETRATION_ALLOWANCE
		}
		return 0.0
	}()) / float64(bodyA.InverseMass+bodyB.InverseMass)) * float64(manifold.Normal.X) * PHYSAC_PENETRATION_CORRECTION)
	correction.Y = float64(((func() float64 {
		if (float64(manifold.Penetration) - PHYSAC_PENETRATION_ALLOWANCE) > 0.0 {
			return float64(manifold.Penetration) - PHYSAC_PENETRATION_ALLOWANCE
		}
		return 0.0
	}()) / float64(bodyA.InverseMass+bodyB.InverseMass)) * float64(manifold.Normal.Y) * PHYSAC_PENETRATION_CORRECTION)
	if bodyA.Enabled {
		bodyA.Position.X -= correction.X * bodyA.InverseMass
		bodyA.Position.Y -= correction.Y * bodyA.InverseMass
	}
	if bodyB.Enabled {
		bodyB.Position.X += correction.X * bodyB.InverseMass
		bodyB.Position.Y += correction.Y * bodyB.InverseMass
	}
}
func GetSupport(shape PhysicsShape, dir Vector2) Vector2 {
	var (
		bestProjection float64     = float64(-PHYSAC_FLT_MAX)
		bestVertex     Vector2     = Vector2{X: 0.0, Y: 0.0}
		data           PolygonData = shape.VertexData
	)
	for i := int(0); i < int(data.VertexCount); i++ {
		var (
			vertex     Vector2 = data.Positions[i]
			projection float64 = MathDot(vertex, dir)
		)
		if projection > bestProjection {
			bestVertex = vertex
			bestProjection = projection
		}
	}
	return bestVertex
}
func FindAxisLeastPenetration(faceIndex *int, shapeA PhysicsShape, shapeB PhysicsShape) float64 {
	var (
		bestDistance float64     = float64(-PHYSAC_FLT_MAX)
		bestIndex    int         = 0
		dataA        PolygonData = shapeA.VertexData
	)
	for i := int(0); i < int(dataA.VertexCount); i++ {
		var (
			normal      Vector2 = dataA.Normals[i]
			transNormal Vector2 = Mat2MultiplyVector2(shapeA.Transform, normal)
			buT         Mat2    = Mat2Transpose(shapeB.Transform)
		)
		normal = Mat2MultiplyVector2(buT, transNormal)
		var support Vector2 = GetSupport(shapeB, Vector2{X: -normal.X, Y: -normal.Y})
		var vertex Vector2 = dataA.Positions[i]
		vertex = Mat2MultiplyVector2(shapeA.Transform, vertex)
		vertex = Vector2Add(vertex, shapeA.Body.Position)
		vertex = Vector2Subtract(vertex, shapeB.Body.Position)
		vertex = Mat2MultiplyVector2(buT, vertex)
		var distance float64 = MathDot(normal, Vector2Subtract(support, vertex))
		if distance > bestDistance {
			bestDistance = distance
			bestIndex = i
		}
	}
	*faceIndex = bestIndex
	return bestDistance
}
func FindIncidentFace(v0 *Vector2, v1 *Vector2, ref PhysicsShape, inc PhysicsShape, index int) {
	var (
		refData         PolygonData = ref.VertexData
		incData         PolygonData = inc.VertexData
		referenceNormal Vector2     = refData.Normals[index]
	)
	referenceNormal = Mat2MultiplyVector2(ref.Transform, referenceNormal)
	referenceNormal = Mat2MultiplyVector2(Mat2Transpose(inc.Transform), referenceNormal)
	var incidentFace int = 0
	var minDot float64 = float64(PHYSAC_FLT_MAX)
	for i := int(0); i < int(incData.VertexCount); i++ {
		var dot float64 = MathDot(referenceNormal, incData.Normals[i])
		if dot < minDot {
			minDot = dot
			incidentFace = i
		}
	}
	*v0 = Mat2MultiplyVector2(inc.Transform, incData.Positions[incidentFace])
	*v0 = Vector2Add(*v0, inc.Body.Position)
	if (incidentFace + 1) < int(incData.VertexCount) {
		incidentFace = incidentFace + 1
	} else {
		incidentFace = 0
	}
	*v1 = Mat2MultiplyVector2(inc.Transform, incData.Positions[incidentFace])
	*v1 = Vector2Add(*v1, inc.Body.Position)
}
func Clip(normal Vector2, clip float64, faceA *Vector2, faceB *Vector2) int {
	var (
		sp        int        = 0
		out       [2]Vector2 = [2]Vector2{*faceA, *faceB}
		distanceA float64    = MathDot(normal, *faceA) - clip
		distanceB float64    = MathDot(normal, *faceB) - clip
	)
	if float64(distanceA) <= 0.0 {
		out[func() int {
			p := &sp
			x := *p
			*p++
			return x
		}()] = *faceA
	}
	if float64(distanceB) <= 0.0 {
		out[func() int {
			p := &sp
			x := *p
			*p++
			return x
		}()] = *faceB
	}
	if float64(distanceA*distanceB) < 0.0 {
		var alpha float64 = distanceA / (distanceA - distanceB)
		out[sp] = *faceA
		var delta Vector2 = Vector2Subtract(*faceB, *faceA)
		delta.X *= alpha
		delta.Y *= alpha
		out[sp] = Vector2Add(out[sp], delta)
		sp++
	}
	*faceA = out[0]
	*faceB = out[1]
	return sp
}
func BiasGreaterThan(valueA float64, valueB float64) bool {
	return float64(valueA) >= (float64(valueB)*0.95 + float64(valueA)*0.01)
}
func TriangleBarycenter(v1 Vector2, v2 Vector2, v3 Vector2) Vector2 {
	var result Vector2 = Vector2{X: 0.0, Y: 0.0}
	result.X = (v1.X + v2.X + v3.X) / 3
	result.Y = (v1.Y + v2.Y + v3.Y) / 3
	return result
}
func InitTimer() {
	rand.Seed(time.Now().Unix())
	frequency = 1000000000
	baseTime = float64(GetTimeCount())
	startTime = GetCurrentTime()
}
func GetTimeCount() int64 {
	return time.Now().UnixNano()
}
func GetCurrentTime() float64 {
	return (float64(GetTimeCount()) - baseTime) / float64(frequency) * 1000
}
func MathCross(value float64, vector Vector2) Vector2 {
	return Vector2{X: -value * vector.Y, Y: value * vector.X}
}
func MathCrossVector2(v1 Vector2, v2 Vector2) float64 {
	return v1.X*v2.Y - v1.Y*v2.X
}
func MathLenSqr(vector Vector2) float64 {
	return vector.X*vector.X + vector.Y*vector.Y
}
func MathDot(v1 Vector2, v2 Vector2) float64 {
	return v1.X*v2.X + v1.Y*v2.Y
}
func DistSqr(v1 Vector2, v2 Vector2) float64 {
	var dir Vector2 = Vector2Subtract(v1, v2)
	return MathDot(dir, dir)
}
func MathNormalize(vector *Vector2) {
	var (
		length  float64
		ilength float64
		aux     Vector2 = *vector
	)
	length = math.Sqrt(aux.X*aux.X + aux.Y*aux.Y)
	if length == 0 {
		length = 1.0
	}
	ilength = float64(1.0 / float64(length))
	vector.X *= ilength
	vector.Y *= ilength
}
func Vector2Add(v1 Vector2, v2 Vector2) Vector2 {
	return Vector2{X: v1.X + v2.X, Y: v1.Y + v2.Y}
}
func Vector2Subtract(v1 Vector2, v2 Vector2) Vector2 {
	return Vector2{X: v1.X - v2.X, Y: v1.Y - v2.Y}
}
func Mat2Radians(radians float64) Mat2 {
	var (
		c float64 = math.Cos(radians)
		s float64 = math.Sin(radians)
	)
	return Mat2{M00: c, M01: -s, M10: s, M11: c}
}
func Mat2Set(matrix *Mat2, radians float64) {
	var (
		cos float64 = math.Cos(radians)
		sin float64 = math.Sin(radians)
	)
	matrix.M00 = cos
	matrix.M01 = -sin
	matrix.M10 = sin
	matrix.M11 = cos
}
func Mat2Transpose(matrix Mat2) Mat2 {
	return Mat2{M00: matrix.M00, M01: matrix.M10, M10: matrix.M01, M11: matrix.M11}
}
func Mat2MultiplyVector2(matrix Mat2, vector Vector2) Vector2 {
	return Vector2{X: matrix.M00*vector.X + matrix.M01*vector.Y, Y: matrix.M10*vector.X + matrix.M11*vector.Y}
}
