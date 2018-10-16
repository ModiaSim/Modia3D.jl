within ;
package PendulumModels
  model Pendulum "Simple pendulum with one revolute joint and one body"
    import Modelica.Mechanics.MultiBody.Frames;
    extends Modelica.Icons.Example;
    inner Modelica.Mechanics.MultiBody.World world(gravityType=Modelica.Mechanics.MultiBody.Types.GravityTypes.
          UniformGravity) annotation (Placement(transformation(extent={{-60,-20},
              {-40,0}})));
    Modelica.Mechanics.MultiBody.Joints.Revolute rev(n={0,0,1},
      phi(fixed=true),
      w(fixed=true),
      useAxisFlange=false)       annotation (Placement(transformation(extent={{-20,-20},
              {0,0}})));
    Modelica.Mechanics.MultiBody.Parts.Body body(r_CM={0.8,0,0}, m=0.5,
      I_11=0,
      I_22=0,
      I_33=0)
      annotation (Placement(transformation(extent={{20,-20},{40,0}})));
    Real P_symbolic "Power computed with symbolic manipulation (must be zero)";
    output Real P_numeric "Power computed with function (must be zero)";
  equation
    P_symbolic = rev.frame_a.f*Frames.resolve2(rev.frame_a.R, der(rev.frame_a.r_0)) +
                 rev.frame_b.f*Frames.resolve2(rev.frame_b.R, der(rev.frame_b.r_0)) +
                 rev.frame_a.t*Frames.angularVelocity2(rev.frame_a.R) +
                 rev.frame_b.t*Frames.angularVelocity2(rev.frame_b.R);
    P_numeric = computePower(rev.frame_a.f, Frames.resolve2(rev.frame_a.R, der(rev.frame_a.r_0)),
                             rev.frame_b.f, Frames.resolve2(rev.frame_b.R, der(rev.frame_b.r_0)),
                             rev.frame_a.t, Frames.angularVelocity2(rev.frame_a.R),
                             rev.frame_b.t, Frames.angularVelocity2(rev.frame_b.R),
                             0.0, time);


    connect(world.frame_b, rev.frame_a)
      annotation (Line(
        points={{-40,-10},{-20,-10}},
        color={95,95,95},
        thickness=0.5));
    connect(body.frame_a, rev.frame_b) annotation (Line(
        points={{20,-10},{0,-10}},
        color={95,95,95},
        thickness=0.5));
    annotation (
      experiment(StopTime=5),
      Documentation(info="<html>
<p>
This simple model demonstrates that by just dragging components
default animation is defined that shows the structure of the
assembled system.

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/Pendulum.png\"
ALT=\"model Examples.Elementary.Pendulum\">
</html>"));
  end Pendulum;

  model PendulumWithDamper
    "Simple pendulum with one revolute joint and one body"
    import Modelica.Mechanics.MultiBody.Frames;
    extends Modelica.Icons.Example;
    inner Modelica.Mechanics.MultiBody.World world(gravityType=Modelica.Mechanics.MultiBody.Types.GravityTypes.
          UniformGravity) annotation (Placement(transformation(extent={{-60,-20},
              {-40,0}})));
    Modelica.Mechanics.MultiBody.Joints.Revolute rev(n={0,0,1},
      phi(fixed=true),
      w(fixed=true),
      useAxisFlange=true)        annotation (Placement(transformation(extent={{-20,-20},
              {0,0}})));
    Modelica.Mechanics.MultiBody.Parts.Body body(r_CM={0.8,0,0}, m=0.5)
      annotation (Placement(transformation(extent={{20,-20},{40,0}})));
    Real P "Power flowing into the revolute joint (must be zero)";
    Modelica.Mechanics.Rotational.Components.Damper damper(d=0.2)
      annotation (Placement(transformation(extent={{-18,18},{2,38}})));
  equation
    P = rev.frame_a.f*Frames.resolve2(rev.frame_a.R, der(rev.frame_a.r_0)) +
        rev.frame_b.f*Frames.resolve2(rev.frame_b.R, der(rev.frame_b.r_0)) +
        rev.frame_a.t*Frames.angularVelocity2(rev.frame_a.R) +
        rev.frame_b.t*Frames.angularVelocity2(rev.frame_b.R) +
        rev.axis.tau*der(rev.axis.phi);

    connect(world.frame_b, rev.frame_a)
      annotation (Line(
        points={{-40,-10},{-20,-10}},
        color={95,95,95},
        thickness=0.5));
    connect(body.frame_a, rev.frame_b) annotation (Line(
        points={{20,-10},{0,-10}},
        color={95,95,95},
        thickness=0.5));
    connect(damper.flange_b, rev.axis) annotation (Line(points={{2,28},{10,28},{10,
            10},{-10,10},{-10,0}}, color={0,0,0}));
    connect(rev.support, damper.flange_a) annotation (Line(points={{-16,0},{-16,10},
            {-28,10},{-28,28},{-18,28}}, color={0,0,0}));
    annotation (
      experiment(StopTime=5),
      Documentation(info="<html>
<p>
This simple model demonstrates that by just dragging components
default animation is defined that shows the structure of the
assembled system.

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/Pendulum.png\"
ALT=\"model Examples.Elementary.Pendulum\">
</html>"));
  end PendulumWithDamper;

  model PendulumWithController
    "Simple pendulum with one revolute joint and one body"
    extends Modelica.Icons.Example;
    inner Modelica.Mechanics.MultiBody.World world(gravityType=Modelica.Mechanics.MultiBody.Types.GravityTypes.
          UniformGravity) annotation (Placement(transformation(extent={{-60,-20},
              {-40,0}})));
    Modelica.Mechanics.MultiBody.Joints.Revolute rev(n={0,0,1},useAxisFlange=true,
      phi(fixed=true),
      w(fixed=true))             annotation (Placement(transformation(extent={{-20,-20},
              {0,0}})));
    Modelica.Mechanics.MultiBody.Parts.Body body(r_CM={0.8,0,0}, m=0.5)
      annotation (Placement(transformation(extent={{20,-20},{40,0}})));
    Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
        Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=-90,
          origin={-28,44})));
    Modelica.Blocks.Sources.Sine sine(amplitude=1.0, freqHz=0.5)
      annotation (Placement(transformation(extent={{-92,70},{-72,90}})));
    Modelica.Blocks.Math.Feedback feedback1
      annotation (Placement(transformation(extent={{-62,70},{-42,90}})));
    Modelica.Mechanics.Rotational.Sources.Torque torque
      annotation (Placement(transformation(extent={{68,70},{88,90}})));
    Modelica.Blocks.Continuous.PI PI(k=10, T=0.01)
      annotation (Placement(transformation(extent={{34,70},{54,90}})));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={10,46})));
    Modelica.Blocks.Math.Gain gain(k=10)
      annotation (Placement(transformation(extent={{-32,70},{-12,90}},
          rotation=0)));
    Modelica.Blocks.Math.Feedback feedback2
      annotation (Placement(transformation(extent={{0,70},{20,90}},    rotation=
           0)));
  equation
    connect(world.frame_b, rev.frame_a)
      annotation (Line(
        points={{-40,-10},{-20,-10}},
        color={95,95,95},
        thickness=0.5));
    connect(body.frame_a, rev.frame_b) annotation (Line(
        points={{20,-10},{0,-10}},
        color={95,95,95},
        thickness=0.5));
    connect(rev.axis, angleSensor.flange)
      annotation (Line(points={{-10,0},{-10,34},{-28,34}},
                                                        color={0,0,0}));
    connect(sine.y, feedback1.u1)
      annotation (Line(points={{-71,80},{-60,80}}, color={0,0,127}));
    connect(angleSensor.phi, feedback1.u2) annotation (Line(points={{-28,55},{
            -28,60},{-52,60},{-52,72}},
                                    color={0,0,127}));
    connect(PI.y, torque.tau)
      annotation (Line(points={{55,80},{66,80}}, color={0,0,127}));
    connect(rev.axis, speedSensor.flange) annotation (Line(points={{-10,0},{-10,
            34},{0,34},{0,36},{10,36}},color={0,0,0}));
    connect(feedback1.y, gain.u) annotation (Line(points={{-43,80},{-38,80},{
            -38,80},{-34,80}}, color={0,0,127}));
    connect(gain.y, feedback2.u1)
      annotation (Line(points={{-11,80},{2,80}}, color={0,0,127}));
    connect(feedback2.y, PI.u)
      annotation (Line(points={{19,80},{32,80}}, color={0,0,127}));
    connect(speedSensor.w, feedback2.u2) annotation (Line(points={{10,57},{10,
            72}},                 color={0,0,127}));
    connect(torque.flange, rev.axis) annotation (Line(points={{88,80},{92,80},{
            92,24},{-10,24},{-10,0}}, color={0,0,0}));
    annotation (
      experiment(StopTime=5),
      Documentation(info="<html>
<p>
This simple model demonstrates that by just dragging components
default animation is defined that shows the structure of the
assembled system.

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/Pendulum.png\"
ALT=\"model Examples.Elementary.Pendulum\">
</html>"));
  end PendulumWithController;

  model Controller "Simple pendulum with one revolute joint and one body"
    extends Modelica.Icons.Example;
    Modelica.Blocks.Sources.Sine sine(amplitude=1.0, freqHz=0.5)
      annotation (Placement(transformation(extent={{-92,-10},{-72,10}})));
    Modelica.Blocks.Math.Feedback feedback1
      annotation (Placement(transformation(extent={{-62,-10},{-42,10}})));
    Modelica.Blocks.Continuous.PI PI(
      k=10,
      T=0.01,
      initType=Modelica.Blocks.Types.Init.InitialState)
      annotation (Placement(transformation(extent={{24,-10},{44,10}})));
    Modelica.Blocks.Math.Gain gain(k=10)
      annotation (Placement(transformation(extent={{-32,-10},{-12,10}},
          rotation=0)));
    Modelica.Blocks.Math.Feedback feedback2
      annotation (Placement(transformation(extent={{-4,-10},{16,10}},  rotation=
           0)));
    RevoluteActuator adapter
      annotation (Placement(transformation(extent={{68,-10},{88,10}})));
    RevConnect revConnect1
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  equation
    connect(sine.y, feedback1.u1)
      annotation (Line(points={{-71,0},{-60,0}}, color={0,0,127}));
    connect(feedback1.y, gain.u)
      annotation (Line(points={{-43,0},{-34,0}}, color={0,0,127}));
    connect(gain.y, feedback2.u1)
      annotation (Line(points={{-11,0},{-2,0}}, color={0,0,127}));
    connect(feedback2.y, PI.u)
      annotation (Line(points={{15,0},{22,0}}, color={0,0,127}));
    connect(PI.y, adapter.tau)
      annotation (Line(points={{45,0},{66,0}}, color={0,0,127}));
    connect(adapter.phi, feedback2.u2) annotation (Line(points={{74,-11},{74,
            -24},{6,-24},{6,-8}}, color={0,0,127}));
    connect(adapter.w, feedback1.u2) annotation (Line(points={{82,-11},{82,-36},
            {-52,-36},{-52,-8}}, color={0,0,127}));
    connect(adapter.revConnect, revConnect1)
      annotation (Line(points={{88,0},{100,0}}, color={28,108,200}));
    annotation (
      experiment(StopTime=5),
      Documentation(info="<html>
<p>
This simple model demonstrates that by just dragging components
default animation is defined that shows the structure of the
assembled system.

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/Pendulum.png\"
ALT=\"model Examples.Elementary.Pendulum\">
</html>"));
  end Controller;

  model RevoluteActuator
    extends Modelica.Blocks.Icons.Block;

    Modelica.Blocks.Interfaces.RealInput tau
      annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
    Modelica.Blocks.Interfaces.RealOutput phi
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={-40,-110})));
    Modelica.Blocks.Interfaces.RealOutput w
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={40,-110})));
    RevConnect revConnect
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  equation
     revConnect.phi = phi;
     revConnect.w   = w;
     revConnect.tau = tau;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Text(
            extent={{-94,-62},{18,-92}},
            lineColor={28,108,200},
            textString="phi"),
          Text(
            extent={{-88,16},{10,-14}},
            lineColor={28,108,200},
            horizontalAlignment=TextAlignment.Left,
            textString="tau"),
          Text(
            extent={{-12,-66},{100,-96}},
            lineColor={28,108,200},
            textString="w")}),                                     Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end RevoluteActuator;

  connector RevConnect
    input Real phi;
    input Real w;
    output Real tau;
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(
            extent={{-40,100},{40,-100}},
            lineColor={28,108,200},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid)}),                      Diagram(
          coordinateSystem(preserveAspectRatio=false), graphics={Ellipse(
            extent={{-40,80},{40,-80}},
            lineColor={28,108,200},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid)}));
  end RevConnect;

  function computePower
    input Real a_f[3];
    input Real a_v[3];
    input Real b_f[3];
    input Real b_v[3];
    input Real a_t[3];
    input Real a_w[3];
    input Real b_t[3];
    input Real b_w[3];
    input Real axis_tau;
    input Real axis_w;
    output Real P;
  protected
     Real P_axis;       // To avoid that function is inlined
  algorithm
     P_axis :=axis_tau*axis_w;
     P :=a_f*a_v + b_f*b_v + a_t*a_w + b_t*b_w + P_axis;
     annotation(Inline = false);
  end computePower;

  model DoublePendulum
    "Simple double pendulum with two revolute joints and two bodies"
    import Modelica.Mechanics.MultiBody.Frames;
    extends Modelica.Icons.Example;
    inner Modelica.Mechanics.MultiBody.World world annotation (Placement(
          transformation(extent={{-88,0},{-68,20}})));
    Modelica.Mechanics.MultiBody.Joints.Revolute rev1(
      useAxisFlange=true,
      phi(fixed=true),
      w(fixed=true))
      annotation (Placement(transformation(extent={{-48,0},{-28,20}})));
    Modelica.Mechanics.Rotational.Components.Damper damper(
                                                d=0.1)
      annotation (Placement(transformation(extent={{-48,40},{-28,60}})));
    Modelica.Mechanics.MultiBody.Parts.BodyBox boxBody1(r={0.5,0,0}, width=0.06)
      annotation (Placement(transformation(extent={{-10,0},{10,20}})));
    Modelica.Mechanics.MultiBody.Joints.Revolute rev2(phi(fixed=true), w(fixed=true))
      annotation (Placement(transformation(extent={{32,0},{52,20}})));
    Modelica.Mechanics.MultiBody.Parts.BodyBox boxBody2(r={0.5,0,0}, width=0.06)
      annotation (Placement(transformation(extent={{74,0},{94,20}})));
    Real P1;
    Real P2;
  equation
      P1 = rev1.frame_a.f*Frames.resolve2(rev1.frame_a.R, der(rev1.frame_a.r_0)) +
           rev1.frame_b.f*Frames.resolve2(rev1.frame_b.R, der(rev1.frame_b.r_0)) +
           rev1.frame_a.t*Frames.angularVelocity2(rev1.frame_a.R) +
           rev1.frame_b.t*Frames.angularVelocity2(rev1.frame_b.R) + rev1.axis.tau*der(rev1.axis.phi);
      P2 = rev2.frame_a.f*Frames.resolve2(rev2.frame_a.R, der(rev2.frame_a.r_0)) +
           rev2.frame_b.f*Frames.resolve2(rev2.frame_b.R, der(rev2.frame_b.r_0)) +
           rev2.frame_a.t*Frames.angularVelocity2(rev2.frame_a.R) +
           rev2.frame_b.t*Frames.angularVelocity2(rev2.frame_b.R);

    connect(damper.flange_b, rev1.axis)
      annotation (Line(points={{-28,50},{-24,50},{-24,28},{-38,28},{-38,20}}));
    connect(rev1.support, damper.flange_a)
      annotation (Line(points={{-44,20},{-44,28},{-58,28},{-58,50},{-48,50}}));
    connect(rev1.frame_b, boxBody1.frame_a) annotation (Line(
        points={{-28,10},{-10,10}},
        color={95,95,95},
        thickness=0.5));
    connect(rev2.frame_b, boxBody2.frame_a) annotation (Line(
        points={{52,10},{74,10}},
        color={95,95,95},
        thickness=0.5));
    connect(boxBody1.frame_b, rev2.frame_a) annotation (Line(
        points={{10,10},{32,10}},
        color={95,95,95},
        thickness=0.5));
    connect(world.frame_b, rev1.frame_a) annotation (Line(
        points={{-68,10},{-48,10}},
        color={95,95,95},
        thickness=0.5));
    annotation (
      experiment(StopTime=3),
      Documentation(info="<html>
<p>
This example demonstrates that by using joint and body
elements animation is automatically available. Also the revolute
joints are animated. Note, that animation of every component
can be switched of by setting the first parameter <b>animation</b>
to <b>false</b> or by setting <b>enableAnimation</b> in the <b>world</b>
object to <b>false</b> to switch off animation of all components.
</p>

<table border=0 cellspacing=0 cellpadding=0><tr><td valign=\"top\">
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/DoublePendulum.png\"
ALT=\"model Examples.Elementary.DoublePendulum\">
</td></tr></table>

</html>"));
  end DoublePendulum;

  model DoublePendulum2
    "Simple double pendulum with two revolute joints and two bodies"
    import Modelica.Mechanics.MultiBody.Frames;
    extends Modelica.Icons.Example;
    inner Modelica.Mechanics.MultiBody.World world annotation (Placement(
          transformation(extent={{-88,0},{-68,20}})));
    Modelica.Mechanics.MultiBody.Joints.Revolute rev1(
      phi(fixed=true),
      w(fixed=true),
      useAxisFlange=false)
      annotation (Placement(transformation(extent={{-48,0},{-28,20}})));
    Modelica.Mechanics.MultiBody.Joints.Revolute rev2(phi(fixed=true), w(fixed=true))
      annotation (Placement(transformation(extent={{32,0},{52,20}})));

    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r={1,0,0})
      annotation (Placement(transformation(extent={{-12,0},{8,20}})));
    Modelica.Mechanics.MultiBody.Parts.Body body1(
      r_CM={0.5,0,0},
      m=1,
      I_11=0,
      I_22=0,
      I_33=0) annotation (Placement(transformation(extent={{-14,28},{6,48}})));
    Modelica.Mechanics.MultiBody.Parts.Body body2(
      r_CM={0.5,0,0},
      m=1,
      I_11=0,
      I_22=0,
      I_33=0) annotation (Placement(transformation(extent={{64,0},{84,20}})));
  equation
    connect(world.frame_b, rev1.frame_a) annotation (Line(
        points={{-68,10},{-48,10}},
        color={95,95,95},
        thickness=0.5));
    connect(rev1.frame_b, fixedTranslation1.frame_a) annotation (Line(
        points={{-28,10},{-20,10},{-12,10}},
        color={95,95,95},
        thickness=0.5));
    connect(fixedTranslation1.frame_b, rev2.frame_a) annotation (Line(
        points={{8,10},{20,10},{32,10}},
        color={95,95,95},
        thickness=0.5));
    connect(rev1.frame_b, body1.frame_a) annotation (Line(
        points={{-28,10},{-24,10},{-24,38},{-14,38}},
        color={95,95,95},
        thickness=0.5));
    connect(rev2.frame_b, body2.frame_a) annotation (Line(
        points={{52,10},{58,10},{64,10}},
        color={95,95,95},
        thickness=0.5));
    annotation (
      experiment(StopTime=5, Tolerance=1e-008),
      Documentation(info="<html>
<p>
This example demonstrates that by using joint and body
elements animation is automatically available. Also the revolute
joints are animated. Note, that animation of every component
can be switched of by setting the first parameter <b>animation</b>
to <b>false</b> or by setting <b>enableAnimation</b> in the <b>world</b>
object to <b>false</b> to switch off animation of all components.
</p>

<table border=0 cellspacing=0 cellpadding=0><tr><td valign=\"top\">
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/DoublePendulum.png\"
ALT=\"model Examples.Elementary.DoublePendulum\">
</td></tr></table>

</html>"));
  end DoublePendulum2;

  model RobotR3
    MechanicalStructure mechanicalStructure
      annotation (Placement(transformation(extent={{-14,-10},{26,30}})));
    inner Modelica.Mechanics.MultiBody.World world(animateGravity=false)
      annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end RobotR3;

  model MechanicalStructure
    "Model of the mechanical part of the r3 robot (without animation)"
    import Modelica.SIunits.Conversions.to_unit1;

    parameter Boolean animation=true "= true, if animation shall be enabled";
    parameter Modelica.SIunits.Mass mLoad(min=0)=15 "Mass of load";
    parameter Modelica.SIunits.Position rLoad[3]={0,0.25,0}
      "Distance from last flange to load mass>";
    parameter Modelica.SIunits.Acceleration g=9.81 "Gravity acceleration";
    Modelica.SIunits.Angle q[6] "Joint angles";
    Modelica.SIunits.AngularVelocity qd[6] "Joint speeds";
    Modelica.SIunits.AngularAcceleration qdd[6] "Joint accelerations";
    Modelica.SIunits.Torque tau[6] "Joint driving torques";
    //r0={0,0.351,0},
    Modelica.Mechanics.Rotational.Interfaces.Flange_a axis1
      annotation (Placement(transformation(extent={{-220,-180},{-200,-160}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a axis2
      annotation (Placement(transformation(extent={{-220,-120},{-200,-100}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a axis3
      annotation (Placement(transformation(extent={{-220,-60},{-200,-40}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a axis4
      annotation (Placement(transformation(extent={{-220,0},{-200,20}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a axis5
      annotation (Placement(transformation(extent={{-220,60},{-200,80}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a axis6
      annotation (Placement(transformation(extent={{-220,120},{-200,140}})));
    inner Modelica.Mechanics.MultiBody.World world(
      g=(g)*Modelica.Math.Vectors.length(
                                    ({0,-1,0})),
      n={0,-1,0},
      animateWorld=false,
      animateGravity=false,
      enableAnimation=animation)
                            annotation (Placement(transformation(extent={{
              -100,-200},{-80,-180}})));
    Modelica.Mechanics.MultiBody.Joints.Revolute r1(n={0,1,0},useAxisFlange=true,
        animation=animation)
      annotation (Placement(transformation(
          origin={-70,-160},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Mechanics.MultiBody.Joints.Revolute r2(n={1,0,0},useAxisFlange=true,
        animation=animation)
      annotation (Placement(transformation(extent={{-50,-110},{-30,-90}})));
    Modelica.Mechanics.MultiBody.Joints.Revolute r3(n={1,0,0},useAxisFlange=true,
        animation=animation)
      annotation (Placement(transformation(
          origin={-50,-36},
          extent={{-10,-10},{10,10}},
          rotation=180)));
    Modelica.Mechanics.MultiBody.Joints.Revolute r4(n={0,1,0},useAxisFlange=true,
        animation=animation)
      annotation (Placement(transformation(
          origin={-70,10},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Mechanics.MultiBody.Joints.Revolute r5(n={1,0,0},useAxisFlange=true,
        animation=animation)
      annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
    Modelica.Mechanics.MultiBody.Joints.Revolute r6(n={0,1,0},useAxisFlange=true,
        animation=animation)
      annotation (Placement(transformation(
          origin={-60,130},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Mechanics.MultiBody.Parts.BodyShape b0(
      r={0,0.351,0},
      r_shape={0,0,0},
      lengthDirection={1,0,0},
      widthDirection={0,1,0},
      length=0.225,
      width=0.3,
      height=0.3,
      color={0,0,255},
      animation=animation,
      animateSphere=false,
      r_CM={0,0,0},
      m=1)
      annotation (Placement(transformation(
          origin={-30,-170},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Mechanics.MultiBody.Parts.BodyShape b1(
      r={0,0.324,0.3},
      I_22=1.16,
      lengthDirection={1,0,0},
      widthDirection={0,1,0},
      length=0.25,
      width=0.15,
      height=0.2,
      animateSphere=false,
      color={255,0,0},
      r_CM={0,0,0},
      m=1,
      animation=false) annotation (Placement(transformation(
          origin={-70,-118},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Mechanics.MultiBody.Parts.BodyShape b2(
      r={0,0.65,0},
      r_CM={0.172,0.205,0},
      m=56.5,
      I_11=2.58,
      I_22=0.64,
      I_33=2.73,
      I_21=-0.46,
      r_shape={0,0,0},
      lengthDirection={1,0,0},
      widthDirection={0,1,0},
      length=0.5,
      width=0.2,
      height=0.15,
      animateSphere=false,
      color={255,178,0},
      animation=false,
      shapeType="sphere")
                         annotation (Placement(transformation(
          origin={-16,-70},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Mechanics.MultiBody.Parts.BodyShape b3(
      r={0,0.414,-0.155},
      r_CM={0.064,-0.034,0},
      m=26.4,
      I_11=0.279,
      I_22=0.245,
      I_33=0.413,
      I_21=-0.070,
      shapeType="modelica://Modelica/Resources/Data/Shapes/RobotR3/b3.dxf",
      r_shape={0,0,0},
      lengthDirection={1,0,0},
      widthDirection={0,1,0},
      length=0.15,
      width=0.15,
      height=0.15,
      animateSphere=false,
      color={255,0,0},
      animation=false) annotation (Placement(transformation(
          origin={-86,-22},
          extent={{-10,10},{10,-10}},
          rotation=90)));
    Modelica.Mechanics.MultiBody.Parts.BodyShape b4(
      r={0,0.186,0},
      r_CM={0,0,0},
      m=28.7,
      I_11=1.67,
      I_22=0.081,
      I_33=1.67,
      shapeType="modelica://Modelica/Resources/Data/Shapes/RobotR3/b4.dxf",
      r_shape={0,0,0},
      lengthDirection={1,0,0},
      widthDirection={0,1,0},
      length=0.73,
      width=0.1,
      height=0.1,
      animateSphere=false,
      color={255,178,0},
      animation=false)   annotation (Placement(transformation(
          origin={-70,50},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Mechanics.MultiBody.Parts.BodyShape b5(
      r={0,0.125,0},
      r_CM={0,0,0},
      m=5.2,
      I_11=1.25,
      I_22=0.81,
      I_33=1.53,
      shapeType="modelica://Modelica/Resources/Data/Shapes/RobotR3/b5.dxf",
      r_shape={0,0,0},
      lengthDirection={1,0,0},
      widthDirection={0,1,0},
      length=0.225,
      width=0.075,
      height=0.1,
      animateSphere=false,
      color={0,0,255},
      animation=false) annotation (Placement(transformation(
          origin={-20,98},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Mechanics.MultiBody.Parts.BodyShape b6(
      r={0,0,0},
      r_CM={0.05,0.05,0.05},
      m=0.5,
      shapeType="modelica://Modelica/Resources/Data/Shapes/RobotR3/b6.dxf",
      r_shape={0,0,0},
      lengthDirection={1,0,0},
      widthDirection={0,1,0},
      animateSphere=false,
      color={0,0,255},
      animation=false) annotation (Placement(transformation(
          origin={-60,160},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Mechanics.MultiBody.Parts.BodyShape load(
      r={0,0,0},
      r_CM=rLoad,
      m=mLoad,
      r_shape={0,0,0},
      widthDirection={1,0,0},
      width=0.05,
      height=0.05,
      color={255,0,0},
      lengthDirection = to_unit1(rLoad),
      length=Modelica.Math.Vectors.length(              rLoad),
      animation=false)
      annotation (Placement(transformation(
          origin={-60,188},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Mechanics.MultiBody.Visualizers.FixedFrame fixedFrame
      annotation (Placement(transformation(extent={{16,-134},{36,-114}})));
    Modelica.Mechanics.MultiBody.Visualizers.FixedFrame fixedFrame1
      annotation (Placement(transformation(extent={{-80,-78},{-60,-58}})));
  equation
    connect(r6.frame_b, b6.frame_a)
      annotation (Line(
        points={{-60,140},{-60,150}},
        color={95,95,95},
        thickness=0.5));
    q = {r1.phi,r2.phi,r3.phi,r4.phi,r5.phi,r6.phi};
    qd = der(q);
    qdd = der(qd);
    tau = {r1.tau, r2.tau, r3.tau, r4.tau, r5.tau, r6.tau};
    connect(load.frame_a, b6.frame_b)
      annotation (Line(
        points={{-60,178},{-60,170}},
        color={95,95,95},
        thickness=0.5));
    connect(world.frame_b, b0.frame_a) annotation (Line(
        points={{-80,-190},{-30,-190},{-30,-180}},
        color={95,95,95},
        thickness=0.5));
    connect(b0.frame_b, r1.frame_a) annotation (Line(
        points={{-30,-160},{-30,-146},{-48,-146},{-48,-180},{-70,-180},{-70,
            -170}},
        color={95,95,95},
        thickness=0.5));
    connect(b1.frame_b, r2.frame_a) annotation (Line(
        points={{-70,-108},{-70,-100},{-50,-100}},
        color={95,95,95},
        thickness=0.5));
    connect(r1.frame_b, b1.frame_a) annotation (Line(
        points={{-70,-150},{-70,-128}},
        color={95,95,95},
        thickness=0.5));
    connect(r2.frame_b, b2.frame_a) annotation (Line(
        points={{-30,-100},{-16,-100},{-16,-80}},
        color={95,95,95},
        thickness=0.5));
    connect(b2.frame_b, r3.frame_a) annotation (Line(
        points={{-16,-60},{-16,-36},{-40,-36}},
        color={95,95,95},
        thickness=0.5));
    connect(r2.axis, axis2) annotation (Line(points={{-40,-90},{-42,-90},{-42,
            -80},{-160,-80},{-160,-110},{-210,-110}}));
    connect(r1.axis, axis1) annotation (Line(points={{-80,-160},{-160,-160},{
            -160,-170},{-210,-170}}));
    connect(r3.frame_b, b3.frame_a) annotation (Line(
        points={{-60,-36},{-88,-36},{-86,-32}},
        color={95,95,95},
        thickness=0.5));
    connect(b3.frame_b, r4.frame_a) annotation (Line(
        points={{-86,-12},{-86,-8},{-70,-8},{-70,0}},
        color={95,95,95},
        thickness=0.5));
    connect(r3.axis, axis3)
      annotation (Line(points={{-50,-46},{-50,-50},{-210,-50}}));
    connect(r4.axis, axis4)
      annotation (Line(points={{-80,10},{-210,10}}));
    connect(r4.frame_b, b4.frame_a)
      annotation (Line(
        points={{-70,20},{-70,40}},
        color={95,95,95},
        thickness=0.5));
    connect(b4.frame_b, r5.frame_a) annotation (Line(
        points={{-70,60},{-70,80},{-60,80}},
        color={95,95,95},
        thickness=0.5));
    connect(r5.axis, axis5) annotation (Line(points={{-50,90},{-50,94},{-160,
            94},{-160,70},{-210,70}}));
    connect(r5.frame_b, b5.frame_a) annotation (Line(
        points={{-40,80},{-20,80},{-20,88}},
        color={95,95,95},
        thickness=0.5));
    connect(b5.frame_b, r6.frame_a) annotation (Line(
        points={{-20,108},{-20,116},{-60,116},{-60,120}},
        color={95,95,95},
        thickness=0.5));
    connect(r6.axis, axis6)
      annotation (Line(points={{-70,130},{-210,130}}));
    connect(b0.frame_b, fixedFrame.frame_a) annotation (Line(
        points={{-30,-160},{-30,-124},{16,-124}},
        color={95,95,95},
        thickness=0.5));
    connect(fixedFrame1.frame_a, r2.frame_a) annotation (Line(
        points={{-80,-68},{-102,-68},{-102,-100},{-50,-100}},
        color={95,95,95},
        thickness=0.5));
    annotation (
      Documentation(info="<html>
<p>
This model contains the mechanical components of the r3 robot
(multibody system).
</p>
</html>"),   Icon(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-200,-200},{200,200}}), graphics={
          Rectangle(
            extent={{-200,200},{200,-200}},
            lineColor={0,0,0},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-200,250},{200,210}},
            textString="%name",
            lineColor={0,0,255}),
          Text(
            extent={{-200,-150},{-140,-190}},
            textString="1",
            lineColor={0,0,255}),
          Text(
            extent={{-200,-30},{-140,-70}},
            textString="3",
            lineColor={0,0,255}),
          Text(
            extent={{-200,-90},{-140,-130}},
            textString="2",
            lineColor={0,0,255}),
          Text(
            extent={{-200,90},{-140,50}},
            textString="5",
            lineColor={0,0,255}),
          Text(
            extent={{-200,28},{-140,-12}},
            textString="4",
            lineColor={0,0,255}),
          Text(
            extent={{-198,150},{-138,110}},
            textString="6",
            lineColor={0,0,255}),
          Bitmap(extent={{-130,-195},{195,195}}, fileName=
                "modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Systems/robot_kr15.png")}),
      Diagram(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-200,-200},{200,200}})));
  end MechanicalStructure;

  model ManyRobots
    RobotR3 robot1
      annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
    RobotR3 robot2
      annotation (Placement(transformation(extent={{-22,38},{-2,58}})));
    RobotR3 robot3
      annotation (Placement(transformation(extent={{18,38},{38,58}})));
    RobotR3 robot4
      annotation (Placement(transformation(extent={{48,36},{68,56}})));
    RobotR3 robot5
      annotation (Placement(transformation(extent={{82,38},{102,58}})));
    RobotR3 robot6
      annotation (Placement(transformation(extent={{-86,-10},{-66,10}})));
    RobotR3 robot7
      annotation (Placement(transformation(extent={{-48,-12},{-28,8}})));
    RobotR3 robot8
      annotation (Placement(transformation(extent={{-8,-12},{12,8}})));
    RobotR3 robot9
      annotation (Placement(transformation(extent={{22,-14},{42,6}})));
    RobotR3 robot10
      annotation (Placement(transformation(extent={{56,-12},{76,8}})));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end ManyRobots;

  model ManyMoreRobots
    ManyRobots manyRobots1
      annotation (Placement(transformation(extent={{-96,40},{-76,60}})));
    ManyRobots manyRobots2
      annotation (Placement(transformation(extent={{-56,40},{-36,60}})));
    ManyRobots manyRobots3
      annotation (Placement(transformation(extent={{-16,40},{4,60}})));
    ManyRobots manyRobots4
      annotation (Placement(transformation(extent={{24,40},{44,60}})));
    ManyRobots manyRobots5
      annotation (Placement(transformation(extent={{64,40},{84,60}})));
    ManyRobots manyRobots6
      annotation (Placement(transformation(extent={{-100,2},{-80,22}})));
    ManyRobots manyRobots7
      annotation (Placement(transformation(extent={{-60,2},{-40,22}})));
    ManyRobots manyRobots8
      annotation (Placement(transformation(extent={{-20,2},{0,22}})));
    ManyRobots manyRobots9
      annotation (Placement(transformation(extent={{20,2},{40,22}})));
    ManyRobots manyRobots10
      annotation (Placement(transformation(extent={{60,2},{80,22}})));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end ManyMoreRobots;

  model ManyMoreRobots50
    ManyRobots manyRobots1 annotation (
      Placement(transformation(extent = {{-96, 40}, {-76, 60}})));
    ManyRobots manyRobots2 annotation (
      Placement(transformation(extent = {{-56, 40}, {-36, 60}})));
    ManyRobots manyRobots3 annotation (
      Placement(transformation(extent = {{-16, 40}, {4, 60}})));
    ManyRobots manyRobots4 annotation (
      Placement(transformation(extent = {{24, 40}, {44, 60}})));
    ManyRobots manyRobots5 annotation (
      Placement(transformation(extent = {{64, 40}, {84, 60}})));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end ManyMoreRobots50;
  annotation (uses(Modelica(version="3.2.2")));
end PendulumModels;
