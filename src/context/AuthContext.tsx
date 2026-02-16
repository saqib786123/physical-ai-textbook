import React, { createContext, useContext, useState, useEffect } from 'react';

type User = {
    username: string;
    token: string;
} | null;

type AuthContextType = {
    user: User;
    login: (userData: User) => void;
    logout: () => void;
};

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export function AuthProvider({ children }: { children: React.ReactNode }) {
    const [user, setUser] = useState<User>(null);

    useEffect(() => {
        const savedUser = localStorage.getItem('auth_user');
        if (savedUser) {
            setUser(JSON.parse(savedUser));
        }
    }, []);

    const login = (userData: User) => {
        setUser(userData);
        if (userData) {
            localStorage.setItem('auth_user', JSON.stringify(userData));
        }
    };

    const logout = () => {
        setUser(null);
        localStorage.removeItem('auth_user');
    };

    return (
        <AuthContext.Provider value={{ user, login, logout }}>
            {children}
        </AuthContext.Provider>
    );
}

export function useAuth() {
    const context = useContext(AuthContext);
    if (undefined === context) {
        throw new Error('useAuth must be used within an AuthProvider');
    }
    return context;
}
