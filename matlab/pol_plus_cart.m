function ret = pol_plus_cart(pol, cart)
    [th2, rh2, z2] = cart2pol(cart(1), cart(2), cart(3));
    [x1, y1, z1] = pol2cart(pol(1) + th2, pol(2) + rh2, pol(3) + z2);
    ret = [x1, y1, z1];
end