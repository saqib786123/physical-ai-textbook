import React, { useState } from 'react';
import { useAuth } from '../../context/AuthContext';
import styles from './styles.module.css';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

type AuthMode = 'signin' | 'signup' | 'forgot';

export default function AuthUI() {
    const { siteConfig } = useDocusaurusContext();
    const backendUrl = siteConfig.customFields?.backendUrl || 'http://localhost:3001';
    const { user, login, logout } = useAuth();
    const [isOpen, setIsOpen] = useState(false);
    const [mode, setMode] = useState<AuthMode>('signin');
    const [formData, setFormData] = useState({ username: '', email: '', password: '' });
    const [error, setError] = useState('');
    const [message, setMessage] = useState('');
    const [loading, setLoading] = useState(false);

    const handleSubmit = async (e: React.FormEvent) => {
        e.preventDefault();
        setError('');
        setMessage('');

        if (mode === 'forgot') {
            setLoading(true);
            try {
                const response = await fetch(`${backendUrl}/auth/forgot`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ email: formData.email }),
                });
                const data = await response.json();
                if (!response.ok) throw new Error(data.message);
                setMessage(data.message);
            } catch (err: any) {
                setError(err.message);
            } finally {
                setLoading(false);
            }
            return;
        }

        setLoading(true);
        const endpoint = mode === 'signup' ? `${backendUrl}/auth/signup` : `${backendUrl}/auth/signin`;

        try {
            const response = await fetch(endpoint, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(formData),
            });

            const data = await response.json();

            if (!response.ok) throw new Error(data.message || 'Something went wrong');

            if (mode === 'signup') {
                setMode('signin');
                setMessage('Signup successful! Please sign in.');
            } else {
                login({ username: data.username, token: data.token });
                setIsOpen(false);
            }
        } catch (err: any) {
            setError(err.message);
        } finally {
            setLoading(false);
        }
    };

    const toggleModal = () => {
        setIsOpen(!isOpen);
        setError('');
        setMessage('');
        setMode('signin');
    };

    if (user) {
        return (
            <div className={styles.userSection}>
                <div className={styles.avatar}>
                    {user.username.charAt(0).toUpperCase()}
                </div>
                <div className={styles.userInfo}>
                    <span className={styles.welcomeText}>{user.username}</span>
                    <button className={styles.logoutBtn} onClick={logout}>Logout</button>
                </div>
            </div>
        );
    }

    return (
        <>
            <button className={styles.floatingAuthBtn} onClick={toggleModal} aria-label="Sign In">
                <svg fill="currentColor" viewBox="0 0 24 24" width="20"><path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm0 3c1.66 0 3 1.34 3 3s-1.34 3-3 3-3-1.34-3-3 1.34-3 3-3zm0 14.2c-2.5 0-4.71-1.28-6-3.22.03-1.99 4-3.08 6-3.08 1.99 0 5.97 1.09 6 3.08-1.29 1.94-3.5 3.22-6 3.22z" /></svg>
                <span>Account</span>
            </button>

            {isOpen && (
                <div className={styles.modalOverlay} onClick={(e) => e.target === e.currentTarget && toggleModal()}>
                    <div className={styles.modalContent}>
                        <button className={styles.closeBtn} onClick={toggleModal}>×</button>
                        <div className={styles.modalHeader}>
                            <div className={styles.logoIcon}>🤖</div>
                            <h2>
                                {mode === 'signin' && 'Welcome Back'}
                                {mode === 'signup' && 'Create Account'}
                                {mode === 'forgot' && 'Reset Password'}
                            </h2>
                            <p className={styles.subtitle}>
                                {mode === 'signin' && 'Access your Physical AI learning portal'}
                                {mode === 'signup' && 'Join the future of humanoid robotics'}
                                {mode === 'forgot' && 'Enter your email to recover access'}
                            </p>
                        </div>

                        <form onSubmit={handleSubmit} className={styles.authForm}>
                            {mode === 'signup' && (
                                <div className={styles.inputGroup}>
                                    <label>Username</label>
                                    <input
                                        type="text"
                                        placeholder="Full Name"
                                        required
                                        value={formData.username}
                                        onChange={(e) => setFormData({ ...formData, username: e.target.value })}
                                    />
                                </div>
                            )}

                            <div className={styles.inputGroup}>
                                <label>Email Address</label>
                                <input
                                    type="email"
                                    placeholder="name@example.com"
                                    required
                                    value={formData.email}
                                    onChange={(e) => setFormData({ ...formData, email: e.target.value })}
                                />
                            </div>

                            {mode !== 'forgot' && (
                                <div className={styles.inputGroup}>
                                    <div className={styles.labelRow}>
                                        <label>Password</label>
                                        {mode === 'signin' && (
                                            <span className={styles.forgotLink} onClick={() => setMode('forgot')}>Forgot?</span>
                                        )}
                                    </div>
                                    <input
                                        type="password"
                                        placeholder="••••••••"
                                        required
                                        value={formData.password}
                                        onChange={(e) => setFormData({ ...formData, password: e.target.value })}
                                    />
                                </div>
                            )}

                            <button type="submit" disabled={loading} className={styles.submitBtn}>
                                {loading ? 'Crunching data...' : (
                                    mode === 'signin' ? 'Sign In' : (mode === 'signup' ? 'Register Now' : 'Send Reset Link')
                                )}
                            </button>
                        </form>

                        {error && <div className={styles.errorAlert}>{error}</div>}
                        {message && <div className={styles.successAlert}>{message}</div>}

                        <div className={styles.modalFooter}>
                            {mode === 'signin' ? (
                                <p>New here? <span onClick={() => setMode('signup')} className={styles.switchLink}>Create an account</span></p>
                            ) : (
                                <p>Already a member? <span onClick={() => setMode('signin')} className={styles.switchLink}>Back to Sign In</span></p>
                            )}
                        </div>
                    </div>
                </div>
            )}
        </>
    );
}
