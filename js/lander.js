function init() {
    var b2Vec2 = Box2D.Common.Math.b2Vec2,
        b2AABB = Box2D.Collision.b2AABB,
        b2BodyDef = Box2D.Dynamics.b2BodyDef,
        b2Body = Box2D.Dynamics.b2Body,
        b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
        b2Fixture = Box2D.Dynamics.b2Fixture,
        b2World = Box2D.Dynamics.b2World,
        b2MassData = Box2D.Collision.Shapes.b2MassData,
        b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
        b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
        b2DebugDraw = Box2D.Dynamics.b2DebugDraw,
        b2MouseJointDef =  Box2D.Dynamics.Joints.b2MouseJointDef,
        canvas = document.getElementById("canvas"),
        scale = 30;
    
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
    
    var world = new b2World(
       new b2Vec2(0, 10),    //gravity
       true                 //allow sleep
    );
    
    var fixDef = new b2FixtureDef();
    fixDef.density = 1.0;
    fixDef.friction = 0.5;
    fixDef.restitution = 0.2;
    
    var bodyDef = new b2BodyDef();
    
    //create ground
    bodyDef.type = b2Body.b2_staticBody;
    fixDef.shape = new b2PolygonShape();
    fixDef.shape.SetAsBox(canvas.width / scale, 2);
    bodyDef.position.Set(10, canvas.height / scale + 1.8);
    world.CreateBody(bodyDef).CreateFixture(fixDef);
    bodyDef.position.Set(10, -1.8);
    world.CreateBody(bodyDef).CreateFixture(fixDef);
    fixDef.shape.SetAsBox(2, canvas.height / scale);
    bodyDef.position.Set(-1.8, 13);
    world.CreateBody(bodyDef).CreateFixture(fixDef);
    bodyDef.position.Set(canvas.width / scale + 1.8, 13);
    world.CreateBody(bodyDef).CreateFixture(fixDef);
    
    //create some objects
    bodyDef.type = b2Body.b2_dynamicBody;
    // for(var i = 0; i < 30; ++i) {
    //     var rnd = Math.random();
    //     if(rnd < 0.333) {
            fixDef.shape = new b2PolygonShape();
            fixDef.shape.SetAsBox(
                Math.random() + 0.1, //half width
                Math.random() + 0.1 //half height
            );
        // } else if(0.666 < rnd) {
        //     fixDef.shape = new b2CircleShape(Math.random() + 0.1);
        // } else {
        //     var triangleWidth = Math.random() + 0.1,
        //         triangleHeight = Math.random() + 0.1;
        //     fixDef.shape = new b2PolygonShape();
        //     // var triangle = [new b2Vec2(0, 0),
        //     //         new b2Vec2(triangleWidth, 0),
        //     //         new b2Vec2(0.5 * triangleWidth, triangleHeight)];
        //     var triangle = [new b2Vec2(0.5 * triangleWidth, 0),
        //             new b2Vec2(0, triangleHeight),
        //             new b2Vec2(triangleWidth, triangleHeight)];
        //     fixDef.shape.SetAsArray(triangle);
        // }
        // bodyDef.position.x = Math.random() * (canvas.width / scale);
        // bodyDef.position.y = Math.random() * (canvas.height / scale);
        bodyDef.position.x = 0.5 * (canvas.width / scale);
        bodyDef.position.y = 0.1;
        var body = world.CreateBody(bodyDef);
        body.CreateFixture(fixDef);
    // }
    
    //setup debug draw
    var debugDraw = new b2DebugDraw();
    debugDraw.SetSprite(canvas.getContext("2d"));
    debugDraw.SetDrawScale(scale);
    debugDraw.SetFillAlpha(0.5);
    debugDraw.SetLineThickness(1.0);
    debugDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit);
    world.SetDebugDraw(debugDraw);
    
    window.setInterval(update, 1000 / 60);
    
    //mouse
    var mouseX, mouseY, mousePVec, isMouseDown, selectedBody, mouseJoint;
    var canvasPosition = getElementPosition(document.getElementById("canvas"));
    
    document.addEventListener("mousedown", function(e) {
        isMouseDown = true;
        handleMouseMove(e);
        document.addEventListener("mousemove", handleMouseMove, true);
    }, true);
    
    document.addEventListener("mouseup", function() {
        document.removeEventListener("mousemove", handleMouseMove, true);
        isMouseDown = false;
        mouseX = undefined;
        mouseY = undefined;
    }, true);
    
    document.addEventListener("keydown", keyDownHandler, true);
    
    function keyDownHandler(e) {
        var keyPressed = String.fromCharCode(e.keyCode);
        if (keyPressed == "W" || keyPressed == "w") {
            var forceVec = new b2Vec2(0, -22);
            forceVec.MulM(body.GetTransform().R);
            body.ApplyForce(forceVec, body.GetWorldCenter());
        } else if (keyPressed == "A" || keyPressed =="a") {
            body.ApplyTorque(-1);
        } else if (keyPressed == "D" || keyPressed =="d") {
            body.ApplyTorque(1);
        }
    }
    
    function handleMouseMove(e) {
        mouseX = (e.clientX - canvasPosition.x) / 30;
        mouseY = (e.clientY - canvasPosition.y) / 30;
    }
    
    function getBodyAtMouse() {
        mousePVec = new b2Vec2(mouseX, mouseY);
        var aabb = new b2AABB();
        aabb.lowerBound.Set(mouseX - 0.001, mouseY - 0.001);
        aabb.upperBound.Set(mouseX + 0.001, mouseY + 0.001);
        
        // Query the world for overlapping shapes.
        selectedBody = null;
        world.QueryAABB(getBodyCB, aabb);
        return selectedBody;
    }
    
    function getBodyCB(fixture) {
        if(fixture.GetBody().GetType() != b2Body.b2_staticBody) {
            if(fixture.GetShape().TestPoint(fixture.GetBody().GetTransform(), mousePVec)) {
                selectedBody = fixture.GetBody();
                return false;
            }
        }
        return true;
    }
    
    //update
    function update() {
        if(isMouseDown && (!mouseJoint)) {
            var body = getBodyAtMouse();
            if(body) {
                var md = new b2MouseJointDef();
                md.bodyA = world.GetGroundBody();
                md.bodyB = body;
                md.target.Set(mouseX, mouseY);
                md.collideConnected = true;
                md.maxForce = 300.0 * body.GetMass();
                mouseJoint = world.CreateJoint(md);
                body.SetAwake(true);
            }
        }
    
        if(mouseJoint) {
            if(isMouseDown) {
                mouseJoint.SetTarget(new b2Vec2(mouseX, mouseY));
            } else {
                world.DestroyJoint(mouseJoint);
                mouseJoint = null;
            }
        }
        
        world.Step(1 / 60, 10, 10);
        world.DrawDebugData();
        world.ClearForces();
    }
    
    //helpers
    //http://js-tut.aardon.de/js-tut/tutorial/position.html
    function getElementPosition(element) {
        var elem=element, tagname="", x=0, y=0;
        
        while((typeof(elem) == "object") && (typeof(elem.tagName) != "undefined")) {
            y += elem.offsetTop;
            x += elem.offsetLeft;
            tagname = elem.tagName.toUpperCase();
            
            if(tagname == "BODY")
                elem=0;
            
            if(typeof(elem) == "object") {
                if(typeof(elem.offsetParent) == "object")
                    elem = elem.offsetParent;
            }
        }
        
        return {x: x, y: y};
    }
}
   
document.onload = new function() {
    init();
};

window.resize = new function() {
    var canvas = document.getElementById("canvas");
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
};
