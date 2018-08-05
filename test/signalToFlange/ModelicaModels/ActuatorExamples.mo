within ;
package ActuatorExamples
  package Components
    model Pendulum "Simple pendulum with one revolute joint and one body"
      extends Modelica.Blocks.Icons.Block;
      inner Modelica.Mechanics.MultiBody.World world(gravityType=Modelica.Mechanics.MultiBody.Types.GravityTypes.
            UniformGravity) annotation (Placement(transformation(extent={{-60,0},{
                -40,20}})));
      Modelica.Mechanics.MultiBody.Joints.Revolute rev(n={0,0,1},useAxisFlange=true)          annotation (Placement(transformation(extent={{
                -20,0},{0,20}})));
      Modelica.Mechanics.MultiBody.Parts.Body body(m=1.0, r_CM={0.5,0,0})
        annotation (Placement(transformation(extent={{20,0},{40,20}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_a axis
        "1-dim. rotational flange that drives the joint"
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
    equation
      connect(world.frame_b, rev.frame_a)
        annotation (Line(
          points={{-40,10},{-20,10}},
          color={95,95,95},
          thickness=0.5));
      connect(body.frame_a, rev.frame_b) annotation (Line(
          points={{20,10},{0,10}},
          color={95,95,95},
          thickness=0.5));
      connect(rev.axis, axis) annotation (Line(points={{-10,20},{-10,34},{-86,34},{-86,
              0},{-100,0}}, color={0,0,0}));
      annotation (
        Documentation(info="<html>
<p>
This simple model demonstrates that by just dragging components
default animation is defined that shows the structure of the
assembled system.

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/Pendulum.png\"
ALT=\"model Examples.Elementary.Pendulum\">
</html>"),
        Icon(graphics={
            Polygon(
              points={{-58,76},{6,76},{-26,50},{-58,76}},
              lineColor={95,95,95},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-26,50},{28,-50}}),
            Ellipse(
              extent={{-4,-14},{60,-78}},
              lineColor={135,135,135},
              fillPattern=FillPattern.Sphere,
              fillColor={255,255,255})}));
    end Pendulum;

    model MoveFlange
        extends Modelica.Blocks.Icons.Block;

      Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Modelica.Mechanics.Rotational.Sources.Position position(useSupport=false,
          exact=true)
        annotation (Placement(transformation(extent={{40,-10},{60,10}})));
      Modelica.Blocks.Sources.Sine sine(amplitude=Modelica.Constants.pi/4, freqHz=1)
        annotation (Placement(transformation(extent={{0,-10},{20,10}})));
    equation
      connect(position.flange, flange_b)
        annotation (Line(points={{60,0},{100,0}}, color={0,0,0}));
      connect(sine.y, position.phi_ref)
        annotation (Line(points={{21,0},{38,0}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Line(points={{-80,0},{-68.7,34.2},{-61.5,53.1},{-55.1,66.4},{-49.4,
                  74.6},{-43.8,79.1},{-38.2,79.8},{-32.6,76.6},{-26.9,69.7},{-21.3,
                  59.4},{-14.9,44.1},{-6.83,21.2},{10.1,-30.8},{17.3,-50.2},{23.7,
                  -64.2},{29.3,-73.1},{35,-78.4},{40.6,-80},{46.2,-77.6},{51.9,-71.5},
                  {57.5,-61.9},{63.9,-47.2},{72,-24.8},{80,0}}, smooth = Smooth.Bezier)}),
                                                                     Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end MoveFlange;

    model MoveFlangeWithFilter
        extends Modelica.Blocks.Icons.Block;

      Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Modelica.Mechanics.Rotational.Sources.Position position(w(fixed=true))
        annotation (Placement(transformation(extent={{40,-10},{60,10}})));
      Modelica.Blocks.Sources.Sine sine(amplitude=Modelica.Constants.pi/4, freqHz=1)
        annotation (Placement(transformation(extent={{0,-10},{20,10}})));
    equation
      connect(position.flange, flange_b)
        annotation (Line(points={{60,0},{100,0}}, color={0,0,0}));
      connect(sine.y, position.phi_ref)
        annotation (Line(points={{21,0},{38,0}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Line(points={{-80,0},{-68.7,34.2},{-61.5,53.1},{-55.1,66.4},{-49.4,
                  74.6},{-43.8,79.1},{-38.2,79.8},{-32.6,76.6},{-26.9,69.7},{-21.3,
                  59.4},{-14.9,44.1},{-6.83,21.2},{10.1,-30.8},{17.3,-50.2},{23.7,
                  -64.2},{29.3,-73.1},{35,-78.4},{40.6,-80},{46.2,-77.6},{51.9,-71.5},
                  {57.5,-61.9},{63.9,-47.2},{72,-24.8},{80,0}}, smooth = Smooth.Bezier),
            Line(points={{-66,0},{-54.7,34.2},{-47.5,53.1},{-41.1,66.4},{-35.4,
                  74.6},{-29.8,79.1},{-24.2,79.8},{-18.6,76.6},{-12.9,69.7},{
                  -7.3,59.4},{-0.9,44.1},{7.17,21.2},{24.1,-30.8},{31.3,-50.2},
                  {37.7,-64.2},{43.3,-73.1},{49,-78.4},{54.6,-80},{60.2,-77.6},
                  {65.9,-71.5},{71.5,-61.9},{77.9,-47.2},{86,-24.8},{94,0}},
                                                                smooth=Smooth.Bezier,
              color={238,46,47})}),                                  Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end MoveFlangeWithFilter;

    model FlangeWithDamper
        extends Modelica.Blocks.Icons.Block;

      Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Modelica.Mechanics.Rotational.Components.Damper damper(d=0.2)
        annotation (Placement(transformation(extent={{22,-10},{42,10}})));
      Modelica.Mechanics.Rotational.Components.Fixed fixed
        annotation (Placement(transformation(extent={{0,-30},{20,-10}})));
    equation
      connect(damper.flange_a, fixed.flange)
        annotation (Line(points={{22,0},{10,0},{10,-20}}, color={0,0,0}));
      connect(damper.flange_b, flange_b)
        annotation (Line(points={{42,0},{100,0}}, color={0,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(extent={{-60,30},{30,-30}},
          lineColor={0,0,0},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid),
        Line(points={{30,0},{90,0}}),
        Line(points={{-60,30},{60,30}}),
        Line(points={{-60,-30},{60,-30}})}),                         Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end FlangeWithDamper;
  end Components;

  model PendulumWithMoveBlock
    "0 states; Pendulum.axis: input: phi, w = der(phi), a = der(w)"
      extends Modelica.Icons.Example;
    Components.Pendulum pendulum    annotation (Placement(transformation(extent={{20,0},{40,20}})));
    Components.MoveFlange           moveFlange
      annotation (Placement(transformation(extent={{-20,0},{0,20}})));
  equation
    connect(moveFlange.flange_b, pendulum.axis)
      annotation (Line(points={{0,10},{20,10}}, color={0,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=3));
  end PendulumWithMoveBlock;

  model PendulumWithMoveBlockAndFilter
    "2 states (position.phi, position.w); Pendulum.axis: input: phi, w = der(phi), a = der(w)"
      extends Modelica.Icons.Example;
    Components.Pendulum pendulum
      annotation (Placement(transformation(extent={{20,0},{40,20}})));
    Components.MoveFlangeWithFilter moveFlangeWithFilter(position(phi(stateSelect=
             StateSelect.always)))
      annotation (Placement(transformation(extent={{-20,0},{0,20}})));
  equation
    connect(moveFlangeWithFilter.flange_b, pendulum.axis)
      annotation (Line(points={{0,10},{20,10}}, color={0,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=3));
  end PendulumWithMoveBlockAndFilter;

  model PendulumWithDamper
    "2 states (pendulum.rev.phi, pendulum.rev.w); Pendulum.axis: output: phi, w = der(phi); input: tau"
      extends Modelica.Icons.Example;
    Components.Pendulum pendulum(rev(
        stateSelect=StateSelect.always,
        phi(fixed=true),
        w(fixed=true)))             annotation (Placement(transformation(extent={{20,0},{40,20}})));
    Components.FlangeWithDamper flangeWithDamper
      annotation (Placement(transformation(extent={{-20,0},{0,20}})));
  equation
    connect(flangeWithDamper.flange_b, pendulum.axis)
      annotation (Line(points={{0,10},{20,10}}, color={0,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=3));
  end PendulumWithDamper;

  model PendulumWithInertia
    "2 states (pendulum.rev.phi, pendulum.rev.w); Pendulum.axis: output: phi, w = der(phi), a = der(w); input: tau"
      extends Modelica.Icons.Example;
    Components.Pendulum pendulum(rev(
        stateSelect=StateSelect.always,
        phi(fixed=true),
        w(fixed=true)))             annotation (Placement(transformation(extent={{20,0},{40,20}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.5)
      annotation (Placement(transformation(extent={{-20,0},{0,20}})));
  equation
    connect(inertia.flange_b, pendulum.axis)
      annotation (Line(points={{0,10},{20,10}}, color={0,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=3));
  end PendulumWithInertia;
  annotation (uses(Modelica(version="3.2.2")));
end ActuatorExamples;
