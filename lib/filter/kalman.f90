!! Copyright (c) 2010, Georgia Tech Research Corporation
!! All rights reserved.
!!
!! Redistribution and use in source and binary forms, with or without
!! modification, are permitted provided that the following conditions
!! are met:
!!
!!     * Redistributions of source code must retain the above
!!       copyright notice, this list of conditions and the following
!!       disclaimer.
!!     * Redistributions in binary form must reproduce the above
!!       copyright notice, this list of conditions and the following
!!       disclaimer in the documentation and/or other materials
!!       provided with the distribution.
!!     * Neither the name of the Georgia Tech Research Corporation nor
!!       the names of its contributors may be used to endorse or
!!       promote products derived from this software without specific
!!       prior written permission.
!!
!! THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
!! IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
!! LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
!! FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
!! TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
!! INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
!! (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
!! SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
!! HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
!! STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
!! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
!! OF THE POSSIBILITY OF SUCH DAMAGE.

!> \file kalman.f90
!! \brief kalman filter
!! \author Neil T. Dantam

Subroutine filter_kalman_predict( n_x, n_u, x, A, B, u, E, R ) 
  Implicit None
  integer, intent(in) :: n_x, n_u                  ! state/input space
  real(8), intent(inout), dimension(n_x) :: x      ! mean state
  real(8), intent(in), dimension(n_x,n_x) :: A     ! process model
  real(8), intent(in), dimension(n_x,n_u) :: B     ! input model
  real(8), intent(in), dimension(n_u) :: u         ! input
  real(8), intent(inout), dimension(n_x,n_x) :: E  ! covariance
  real(8), intent(in), dimension(n_x,n_x) ::  R    ! noise model
  ! x = A*x + B*u
  x = matmul(A,x) + matmul(B,u)
  ! E = A * E * A**T + R
  E = matmul( matmul(A,E), transpose(A) ) + R
End Subroutine filter_kalman_predict

Function filter_kalman_correct(n_x, n_z, x, C, z, E, Q ) result(info)
  Implicit None
  !! interface to inversion function (via lapack) in amino
  Interface
     Function aa_la_inv( n, A ) result(info)
       integer, intent(in) :: n
       real(8), dimension(n,n), intent(inout) :: A
       integer :: info
     End Function aa_la_inv
  End Interface
  integer, intent(in) :: n_x, n_z                  ! state, measurement space
  real(8), intent(inout), dimension(n_x) :: x      ! state
  real(8), intent(in), dimension(n_z) :: z         ! measurement
  real(8), intent(in), dimension(n_z, n_x) :: C    ! measurement model
  real(8), intent(inout), dimension(n_x, n_x) :: E ! covariance
  real(8), intent(in), dimension(n_z, n_z) :: Q    ! measurement noise
  integer :: info

  real(8), dimension( n_x, n_z ) :: K
  real(8), dimension( n_z, n_z ) :: Kp
  real(8), dimension( n_x, n_x ) :: Ident
  integer :: i

  ! K = E * C**T * (C * E * C**T + Q)**-1
  Kp = matmul( matmul(C, E), transpose(C) ) + Q
  info = aa_la_inv(n_z, Kp)
  K = matmul( matmul(E, transpose(C)), Kp )

  ! x = x + K * (z - C*x)
  x = x + matmul( K, z - matmul(C,x) )

  ! E = (I - K*C) * E 
  Ident = 0
  Forall ( i = 1:n_x )
     Ident(i,i) = 1
  End Forall
  E = matmul( Ident - matmul(K,C), E  )
End Function filter_kalman_correct
