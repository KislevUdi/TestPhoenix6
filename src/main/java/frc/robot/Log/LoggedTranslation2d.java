// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Log;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;

import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

/** Add your docs here. */
public class LoggedTranslation2d extends Translation2d {

    private double x;
    private double y;

    public LoggedTranslation2d(String name, double x,double y) {
      set(x,y);
      LogManager.addEntry(name + "/x", this::getX);
      LogManager.addEntry(name + "/y", this::getY);
  }

  public LoggedTranslation2d(String name, double distance, Rotation2d angle) {
    super(distance * angle.getCos(), distance * angle.getSin());
  }

  public void set(double x, double y) {
    this.x = x;
    this.y = y;
  }

  public void set(Translation2d translation) {
    set(translation.getX(), translation.getY());
  }
  /**
   * Calculates the distance between two translations in 2D space.
   *
   * <p>The distance between translations is defined as √((x₂−x₁)²+(y₂−y₁)²).
   *
   * @param other The translation to compute the distance to.
   * @return The distance between the two translations.
   */
  public double getDistance(Translation2d other) {
    return Math.hypot(other.getX() - x ,other.getY() - y);
  }

  /**
   * Returns the X component of the translation.
   *
   * @return The X component of the translation.
   */
  @JsonProperty
  public double getX() {
    return x;
  }

  /**
   * Returns the Y component of the translation.
   *
   * @return The Y component of the translation.
   */
  @JsonProperty
  public double getY() {
    return y;
  }

  /**
   * Returns a vector representation of this translation.
   *
   * @return A Vector representation of this translation.
   */
  public Vector<N2> toVector() {
    return VecBuilder.fill(x, y);
  }

  /**
   * Returns the norm, or distance from the origin to the translation.
   *
   * @return The norm of the translation.
   */
  public double getNorm() {
    return Math.hypot(x,y);
  }

  /**
   * Returns the angle this translation forms with the positive X axis.
   *
   * @return The angle of the translation
   */
  public Rotation2d getAngle() {
    return new Rotation2d(x, y);
  }

  /**
   * Applies a rotation to the translation in 2D space.
   *
   * <p>This multiplies the translation vector by a counterclockwise rotation matrix of the given
   * angle.
   *
   * <pre>
   * [x_new]   [other.cos, -other.sin][x]
   * [y_new] = [other.sin,  other.cos][y]
   * </pre>
   *
   * <p>For example, rotating a Translation2d of &lt;2, 0&gt; by 90 degrees will return a
   * Translation2d of &lt;0, 2&gt;.
   *
   * @param other The rotation to rotate the translation by.
   * @return The new rotated translation.
   */
  public Translation2d rotateBy(Rotation2d other) {
    return new Translation2d(
        x * other.getCos() - y * other.getSin(), x * other.getSin() +y * other.getCos());
  }
  public void rotate(Rotation2d other) {
    set(x * other.getCos() - y * other.getSin(), x * other.getSin() +y * other.getCos());
  }

  /**
   * Returns the sum of two translations in 2D space.
   *
   * <p>For example, Translation3d(1.0, 2.5) + Translation3d(2.0, 5.5) = Translation3d{3.0, 8.0).
   *
   * @param other The translation to add.
   * @return The sum of the translations.
   */
  public Translation2d plus(Translation2d other) {
    return new Translation2d(x + other.getX(), y + other.getY());
  }
  public void add(Translation2d other) {
    set(x + other.getX(), y + other.getY());
  }

  /**
   * Returns the difference between two translations.
   *
   * <p>For example, Translation2d(5.0, 4.0) - Translation2d(1.0, 2.0) = Translation2d(4.0, 2.0).
   *
   * @param other The translation to subtract.
   * @return The difference between the two translations.
   */
  public Translation2d minus(Translation2d other) {
    return new Translation2d(x - other.getY(), y - other.getY());
  }
  public void substract(Translation2d other) {
    set(x - other.getY(), y - other.getY());
  }

  /**
   * Returns the inverse of the current translation. This is equivalent to rotating by 180 degrees,
   * flipping the point over both axes, or negating all components of the translation.
   *
   * @return The inverse of the current translation.
   */
  public Translation2d unaryMinus() {
    return new Translation2d(-x, -y);
  }
  public void reverse() {
    set(-x, -y);
  }

  /**
   * Returns the translation multiplied by a scalar.
   *
   * <p>For example, Translation2d(2.0, 2.5) * 2 = Translation2d(4.0, 5.0).
   *
   * @param scalar The scalar to multiply by.
   * @return The scaled translation.
   */
  public Translation2d times(double scalar) {
    return new Translation2d(x * scalar, y * scalar);
  }
  public void mult(double scalar) {
    set(x * scalar, y * scalar);
  }

  /**
   * Returns the translation divided by a scalar.
   *
   * <p>For example, Translation3d(2.0, 2.5) / 2 = Translation3d(1.0, 1.25).
   *
   * @param scalar The scalar to multiply by.
   * @return The reference to the new mutated object.
   */
  public Translation2d div(double scalar) {
    return new Translation2d(x / scalar, y / scalar);
  }
  public void divide(double scalar) {
    set(x / scalar, y / scalar);
  }

  /**
   * Returns the nearest Translation2d from a list of translations.
   *
   * @param translations The list of translations.
   * @return The nearest Translation2d from the list.
   */
  public Translation2d nearest(List<Translation2d> translations) {
    return Collections.min(translations, Comparator.comparing(this::getDistance));
  }

  @Override
  public String toString() {
    return String.format("Translation2d(X: %.2f, Y: %.2f)", x, y);
  }

  /**
   * Checks equality between this Translation2d and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    if (obj instanceof Translation2d) {
      Translation2d t = (Translation2d) obj; 
      return Math.abs(t.getX() - x) < 1E-9
          && Math.abs(t.getY() - y) < 1E-9;
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(x, y);
  }

  @Override
  public Translation2d interpolate(Translation2d endValue, double t) {
    return new Translation2d(
        MathUtil.interpolate(this.getX(), endValue.getX(), t),
        MathUtil.interpolate(this.getY(), endValue.getY(), t));
  }

}

