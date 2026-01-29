# Transform Nomenclature

---

## Exercise sheet

<p align="center">
  <img src="../recursos/imgs/hw1excercise.jpg" width="650">
</p>

---

## 1) Vector rotation (active rotations)

**Problem:** A vector **A** is rotated about **Ŷ_A** by **45°**, and then rotated about **X̂_A** by **60°**.  
Using **active rotations** (rotating the vector) with **fixed axes in {A}**, the order “first Y, then X” is:

\[
\mathbf{R} = \mathbf{R}_x(60^\circ)\,\mathbf{R}_y(45^\circ)
\]

### Rotation order

---

### Rotation about Yₐ (45°)

**Rᵧ(45°)**

| cos45 | 0 | sin45 |
|------:|--:|------:|
| 0 | 1 | 0 |
| −sin45 | 0 | cos45 |

---

### Rotation about Xₐ (60°)

**Rₓ(60°)**

| 1 | 0 | 0 |
|--:|--:|--:|
| 0 | cos60 | −sin60 |
| 0 | sin60 | cos60 |

---

### Final rotation matrix

**R = Rₓ(60°) · Rᵧ(45°)**

| √2/2 | 0 | √2/2 |
|-----:|--:|-----:|
| √6/4 | 1/2 | −√6/4 |
| −√2/4 | √3/2 | √2/4 |

!!! tip "Numerical values"
    cos45 = sin45 = √2/2  
    cos60 = 1/2  
    sin60 = √3/2  

---

## 2) Frame {B} rotated w.r.t. {A} + translation

**Given:**
- Rotation about **X̂ₐ = 30°**
- Translation vector **p**

| x | y | z |
|--:|--:|--:|
| 5 | 10 | 0 |

---

### Rotation matrix ᴬRᴮ = Rₓ(30°)

| 1 | 0 | 0 |
|--:|--:|--:|
| 0 | √3/2 | −1/2 |
| 0 | 1/2 | √3/2 |

---

### Homogeneous transformation ᴬTᴮ

| 1 | 0 | 0 | 5 |
|--:|--:|--:|--:|
| 0 | √3/2 | −1/2 | 10 |
| 0 | 1/2 | √3/2 | 0 |
| 0 | 0 | 0 | 1 |

---

## 3) From the diagram

---

### (a) Homogeneous transformation ᴬTᴮ

**Translation vector**

| x | y | z |
|--:|--:|--:|
| 3 | 0 | 0 |

---

**Rotation matrix ᴬRᴮ**

| −1 | 0 | 0 |
|---:|--:|--:|
| 0 | −1 | 0 |
| 0 | 0 | 1 |

---

**Homogeneous matrix ᴬTᴮ**

| −1 | 0 | 0 | 3 |
|---:|--:|--:|--:|
| 0 | −1 | 0 | 0 |
| 0 | 0 | 1 | 0 |
| 0 | 0 | 0 | 1 |

---

### (b) Homogeneous transformation ᴬTᶜ

**Translation vector**

| x | y | z |
|--:|--:|--:|
| 3 | 0 | 2 |

---

**Unit axes of frame C expressed in frame A**

**x̂ᶜ**

| 0 |
|--:|
| 0 |
| −1 |

**ŷᶜ**

| −1/2 |
|-----:|
| √3/2 |
| 0 |

**ẑᶜ = x̂ᶜ × ŷᶜ**

| √3/2 |
|-----:|
| 1/2 |
| 0 |

---

### Rotation matrix ᴬRᶜ  
(columns = x̂ᶜ, ŷᶜ, ẑᶜ)

| 0 | −1/2 | √3/2 |
|--:|-----:|-----:|
| 0 | √3/2 | 1/2 |
| −1 | 0 | 0 |

---

### Homogeneous transformation ᴬTᶜ

| 0 | −1/2 | √3/2 | 3 |
|--:|-----:|-----:|--:|
| 0 | √3/2 | 1/2 | 0 |
| −1 | 0 | 0 | 2 |
| 0 | 0 | 0 | 1 |
